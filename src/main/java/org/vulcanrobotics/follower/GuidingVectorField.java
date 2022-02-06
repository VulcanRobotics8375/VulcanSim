package org.vulcanrobotics.follower;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.BisectionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.apache.commons.math3.util.FastMath;
import org.vulcanrobotics.App;
import org.vulcanrobotics.math.control.PID;
import org.vulcanrobotics.math.geometry.ArcLength;
import org.vulcanrobotics.math.geometry.Distance;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;
import org.vulcanrobotics.math.utils.Angle;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;
import org.vulcanrobotics.sim.motors.ZeroPowerBehavior;

import java.util.ArrayList;

public class GuidingVectorField extends Follower {

    public static double CROSS_TRACK_ERROR_GAIN = 1.0;
    public static double TANGENT_VECTOR_GAIN = 1.0;

    Mecanum model;
    NeverestOrbital20 fl, fr, bl, br;

    PolynomialSplineFunction spline;
    SplineInterpolator interpolator = new SplineInterpolator();
    ArrayList<Pose> guidePoints;

    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    BisectionSolver solver = new BisectionSolver();
    Distance distance;
    ArcLength arcLength;
    double robotDistanceTravelled;

    PID headingPID = new PID(0.4, 0.0, 0.0, -1.0, 1.0);

    public GuidingVectorField(Path path, ArrayList<Pose> guidePoints) {
        super(path);
        this.guidePoints = guidePoints;


        // initialize motors and robot model
        fl = new NeverestOrbital20();
        fr = new NeverestOrbital20();
        bl = new NeverestOrbital20();
        br = new NeverestOrbital20();
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); //FLOAT doesnt work yet, things may break if you use it
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        model = new Mecanum(12, 8, 1.887, 30, new Motor[] {fl, fr, bl, br});
        setModel(model);

        double[] x = new double[guidePoints.size()];
        double[] y = new double[guidePoints.size()];
        for (int i = 0; i < guidePoints.size(); i++) {
            Pose p = guidePoints.get(i);
            x[i] = p.x;
            y[i] = p.y;
        }
        spline = interpolator.interpolate(x, y);
        distance = new Distance(spline);
        arcLength = new ArcLength(spline);
        robotDistanceTravelled = 0.5;
        App.drawFunction(spline, 0, spline.getKnots()[spline.getN()]);
    }

    @Override
    public void run() {
        Pose robotPose = model.getRobotPose();

        //robot arc length calculation
        Pose robotVelocity = model.getRobotPoseVelocity();
        robotDistanceTravelled += Math.sqrt((robotVelocity.x * robotVelocity.x) + (robotVelocity.y * robotVelocity.y));

        distance.updatePos(robotPose.vector());
        double currentX = solver.solve(100, x -> arcLength.value(x) - robotDistanceTravelled, 0.1, spline.getKnots()[spline.getN()]);
        UnivariatePointValuePair minValue = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, new SearchInterval(currentX, spline.getKnots()[spline.getN()]));

        double derivative = spline.derivative().value(minValue.getPoint());
        double derivativeHeading = FastMath.atan2(derivative, 1);
        Vector tangentVec = new Vector(FastMath.cos(derivativeHeading), FastMath.sin(derivativeHeading)).multiply(TANGENT_VECTOR_GAIN);
        Vector crossTrackVec = new Vector(minValue.getPoint() - robotPose.x, spline.value(minValue.getPoint()) - robotPose.y).multiply(CROSS_TRACK_ERROR_GAIN);

        Vector resultant = tangentVec.plus(crossTrackVec);
        double vectorAngle = tangentVec.angle();
        double turnOutput = headingPID.run(Angle.diff(robotPose.heading, vectorAngle));
        double translationalVectorScalar = 1.0 - (2.0 * Math.abs(turnOutput));
        resultant = resultant.multiply(translationalVectorScalar);

        double[] outputWheelVelocities = model.calculateWheelVelocities(MatrixUtils.createColumnRealMatrix(new double[]{resultant.x, resultant.y, turnOutput}));
        setPowers(outputWheelVelocities);
    }

    void setPowers(double[] powers) {
        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);

    }
}
