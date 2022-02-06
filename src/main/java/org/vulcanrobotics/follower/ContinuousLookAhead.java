package org.vulcanrobotics.follower;

import org.apache.commons.math3.analysis.UnivariateFunction;
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
import org.vulcanrobotics.math.utils.Angle;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;
import org.vulcanrobotics.sim.motors.ZeroPowerBehavior;

import java.util.ArrayList;

public class ContinuousLookAhead extends Follower{
    Mecanum model;
    NeverestOrbital20 fl, fr, bl, br;

    final double lookAhead = 2;

    PolynomialSplineFunction spline;
    SplineInterpolator interpolator = new SplineInterpolator();
    ArrayList<Pose> guidePoints;

    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    BisectionSolver solver = new BisectionSolver();
    Distance distance;
    ArcLength arcLength;
    ArcLengthZero arcLengthZero;

    PID headingPID = new PID(0.3, 0.0, 0.0, -1.0, 1.0);

    public ContinuousLookAhead(Path path, ArrayList<Pose> guidePoints) {
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
        arcLengthZero = new ArcLengthZero();

        App.drawFunction(spline, 0, spline.getKnots()[spline.getN()]);
    }

    public void run() {
        Pose robotPose = model.getRobotPose();

        distance.updatePos(robotPose.vector());
        double targetX = 0;
        double targetY = 0;
        try {
            UnivariatePointValuePair min = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, new SearchInterval(0, spline.getKnots()[spline.getN()]));

            arcLength.updateStartX(min.getPoint());

            targetX = solver.solve(100, arcLengthZero, min.getPoint() + 1, spline.getKnots()[spline.getN()]);
            targetY = spline.value(targetX);
        } catch (Exception e) {
            e.printStackTrace();
        }

        Pose targetPose = new Pose(targetX, targetY, 0.0);
        double angleToPoint = Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x);
        double turnOutput = headingPID.run(angleToPoint, robotPose.heading);
        Pose velocityOut = new Pose(FastMath.cos(angleToPoint), FastMath.sin(angleToPoint), turnOutput);

        double[] outputWheelVelocities = model.calculateWheelVelocities(MatrixUtils.createColumnRealMatrix(new double[]{velocityOut.x, velocityOut.y, velocityOut.heading}));

        setPowers(outputWheelVelocities);
    }

    void setPowers(double[] powers) {
        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);

    }

    private class ArcLengthZero implements UnivariateFunction {
        @Override
        public double value(double x) {
            return ContinuousLookAhead.this.arcLength.value(x) - ContinuousLookAhead.this.lookAhead;
        }
    }
}
