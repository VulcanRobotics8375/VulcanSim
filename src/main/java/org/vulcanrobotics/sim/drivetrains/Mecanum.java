package org.vulcanrobotics.sim.drivetrains;

import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.MotorType;

public class Mecanum extends RobotModel {

    private final RealMatrix velocityMatrix;
    private final RealMatrix wheelVelocityMatrix;

    public Mecanum(double wheelBaseY, double wheelBaseX, double wheelRadius, double robotWeight, Motor[] motors) {
        super(wheelRadius, robotWeight, motors);
        wheelVelocityMatrix = constructMecanumWheelVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);
        velocityMatrix = constructMecanumVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);
        if(motors.length != 4) {
            //TODO error handler of some sort with UI to get rid of console error logs like this
            System.out.println("mecanum needs 4 motors in its configuration");
        }
    }

    @Override
    public void update(double... powers) throws Exception {
        if(powers.length != 4) {
            throw new Exception("mecanum requires 4 motor powers");
        }

        //give motors their target velocity, let motor physics API do the rest?
        //maybe let RobotModel handle physics. Motor objects should just be for motor intrinsics

    }

    public Pose calculateRobotVelocity(RealMatrix wheelVelocities) {
        RealMatrix velMatrix = velocityMatrix.multiply(wheelVelocities);
        return new Pose(velMatrix.getEntry(0, 0), velMatrix.getEntry(1, 0), velMatrix.getEntry(2, 0));
    }

    public double[] calculateWheelVelocities(RealMatrix poseVelocity) {
        RealMatrix m = wheelVelocityMatrix.multiply(poseVelocity);
        return new double[] {
                m.getEntry(0, 0),
                m.getEntry(1, 0),
                m.getEntry(2, 0),
                m.getEntry(3, 0)
        };
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
