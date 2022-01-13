package org.vulcanrobotics.sim.drivetrains;

import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;
import org.vulcanrobotics.sim.RobotModel;

public class Mecanum extends RobotModel {

    private RealMatrix velocityMatrix;
    private RealMatrix wheelVelocityMatrix;

    public Mecanum(double wheelBaseY, double wheelBaseX, double wheelRadius) {
        wheelVelocityMatrix = constructMecanumWheelVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);
        velocityMatrix = constructMecanumVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);



    }

    @Override
    public void update(double... powers) throws Exception {
        if(powers.length != 4) {
            throw new Exception("mecanum requires 4 motor powers");
        }

        Pose targetPoseVelocity = calculateRobotVelocity(MatrixUtils.createRowRealMatrix(powers));

    }

    public Pose calculateRobotVelocity(RealMatrix wheelVelocities) {
        RealMatrix velMatrix = velocityMatrix.multiply(wheelVelocities);
        return new Pose(velMatrix.getEntry(0, 0), velMatrix.getEntry(1, 0), velMatrix.getEntry(2, 0));
    }

    //TODO make this work with any motor order
    private RealMatrix constructMecanumVelocityMatrix(double wheelBaseY, double wheelBaseX, double wheelRadius) {

        double[][] matrixVals = new double[][] {
                {1.0, 1.0, 1.0, 1.0},
                {1.0, -1.0, -1.0, 1.0},
                {-1.0 / (wheelBaseY + wheelBaseX), 1.0 / (wheelBaseY + wheelBaseX), -1.0 / (wheelBaseY + wheelBaseX), 1.0 / (wheelBaseY + wheelBaseX)}
        };

       return MatrixUtils.createRealMatrix(matrixVals).scalarMultiply(wheelRadius / 4.0);
    }

    private RealMatrix constructMecanumWheelVelocityMatrix(double wheelBaseY, double wheelBaseX, double wheelRadius) {
        double[][] matrixVals = new double[][] {
                {1.0, 1.0, -(wheelBaseY + wheelBaseX)},
                {1.0, -1.0, (wheelBaseY + wheelBaseX)},
                {1.0, -1.0, -(wheelBaseY + wheelBaseX)},
                {1.0, 1.0, (wheelBaseY + wheelBaseX)}
        };

        return MatrixUtils.createRealMatrix(matrixVals).scalarMultiply(1.0 / wheelRadius);
    }

    //TODO for matt or soham to do

}
