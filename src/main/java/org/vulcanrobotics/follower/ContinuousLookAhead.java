package org.vulcanrobotics.follower;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BisectionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.apache.commons.math3.util.FastMath;
import org.vulcanrobotics.math.geometry.ArcLength;
import org.vulcanrobotics.math.geometry.Distance;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;
import org.vulcanrobotics.sim.motors.ZeroPowerBehavior;

public class ContinuousLookAhead extends Follower{
    Mecanum model;
    NeverestOrbital20 fl, fr, bl, br;

    final double lookAhead = 10;

    UnivariateFunction function = new Function();
    Distance distance = new Distance(function);
    ArcLength arcLength = new ArcLength(function);
    ArcLengthZero arcLengthZero = new ArcLengthZero();

    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    BisectionSolver solver = new BisectionSolver();

    public ContinuousLookAhead(Path path) {
        super(path);
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
    }

    public void run() {
        Pose robotPose = model.getRobotPose();

        distance.updatePos(robotPose.vector());
        double targetX = 0;
        double targetY = 0;
        try {
            UnivariatePointValuePair min = optim.optimize(new MaxEval(1000), new UnivariateObjectiveFunction(distance), GoalType.MINIMIZE, new SearchInterval(-1000, 1000));
//            System.out.println("(" + min.getPoint() + ", " + min.getValue() + ")");
            arcLength.updateStartX(min.getPoint());

            targetX = solver.solve(100, arcLengthZero, min.getPoint() + 1, min.getPoint() + 1000);
            targetY = function.value(targetX);
        } catch (Exception e) {
            e.printStackTrace();
        }

        Pose targetPose = new Pose(targetX, targetY, 0.0);
        double angleToPoint = Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x);
        Pose velocityOut = new Pose(Math.cos(angleToPoint), Math.sin(angleToPoint), 0.0);

//        System.out.println("Target: " + String.format("%.2f", targetPose.x) + ", " + String.format("%.2f", targetPose.y));
//        System.out.println("Current: " + String.format("%.2f", robotPose.x) + ", " + String.format("%.2f", robotPose.y));
//        System.out.println("Velocity: " + String.format("%.2f", velocityOut.x) + ", " + String.format("%.2f", velocityOut.y));

        double[] outputWheelVelocities = model.calculateWheelVelocities(MatrixUtils.createColumnRealMatrix(new double[]{velocityOut.x, velocityOut.y, velocityOut.heading}));

        setPowers(outputWheelVelocities);
    }

    void setPowers(double[] powers) {
        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);

    }

    private static class Function implements UnivariateFunction {
        @Override
        public double value(double x) {
            return FastMath.pow(x / 10, 2);
        }
    }
    private class ArcLengthZero implements UnivariateFunction {
        @Override
        public double value(double x) {
            return ContinuousLookAhead.this.arcLength.value(x) - ContinuousLookAhead.this.lookAhead;
        }
    }
}
