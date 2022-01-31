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

public class ContinuousLookAhead extends Follower{
    Mecanum model;

    final double lookAhead = 10;

    UnivariateFunction function = new Function();
    Distance distance = new Distance(function);
    ArcLength arcLength = new ArcLength(function);
    ArcLengthZero arcLengthZero = new ArcLengthZero();

    BrentOptimizer optim = new BrentOptimizer(1e-10, 1e-14);
    BisectionSolver solver = new BisectionSolver();

    public ContinuousLookAhead(Path path, Mecanum model) {
        super(path, model);
        this.model = model;
    }

    public void run() throws Exception {
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

        model.update(outputWheelVelocities);
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
