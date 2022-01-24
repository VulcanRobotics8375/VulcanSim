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

    private final double maxWheelAccel;
    private double robotWeight;

    private final Motor[] motors;

    public Mecanum(double wheelBaseY, double wheelBaseX, double wheelRadius, double robotWeight, Motor[] motors) {
        wheelVelocityMatrix = constructMecanumWheelVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);
        velocityMatrix = constructMecanumVelocityMatrix(wheelBaseY, wheelBaseX, wheelRadius);
        if(motors.length != 4) {
            //TODO error handler of some sort with UI to get rid of console error logs like this
            System.out.println("mecanum needs 4 motors in its configuration");
        }
        this.motors = motors;
        this.robotWeight = robotWeight;
        maxWheelAccel = calculateMaxWheelAccel(wheelRadius, robotWeight, motors[0].getMotorProperties());
        this.wheelRadius = wheelRadius;
        System.out.println(calculateMaxFrictionAccelRad());
    }

    @Override
    public void update(double... powers) throws Exception {
        if(powers.length != 4) {
            throw new Exception("mecanum requires 4 motor powers");
        }
        //setup target velocities
        //might be able to make these local variables depending on how this goes
        Pose targetVelocity = calculateRobotVelocity(MatrixUtils.createColumnRealMatrix(powers));

        double[] currentWheelVelocities = new double[] {
                motors[0].getSpeed(),
                motors[1].getSpeed(),
                motors[2].getSpeed(),
                motors[3].getSpeed()
        };

        for(int i = 0; i < 4; i++) {
            double targetAccel = (powers[i] - currentWheelVelocities[i]);
            double allowedWheelAccel = Math.abs(targetAccel) > (maxWheelAccel * loopTime) ? (maxWheelAccel * Math.signum(targetAccel) * loopTime) : targetAccel;
            double resistiveWheelAccel = robotWeight * motors[i].getMotorProperties().backDriveTorque() * currentWheelVelocities[i] * -1.0;
            double wheelForceOutput = allowedWheelAccel + resistiveWheelAccel;
            currentWheelVelocities[i] += wheelForceOutput;
            motors[i].speed(currentWheelVelocities[i]);
        }

        Pose velocityUpdate = calculateRobotVelocity(MatrixUtils.createColumnRealMatrix(currentWheelVelocities).scalarMultiply(loopTime * (motors[0].getMotorProperties().maxRPM()) * (2.0 * Math.PI / 60.0)));
        setRobotPose(robotPose.plus(velocityUpdate));
        System.out.println(velocityUpdate);
    }

    private double calculateMaxWheelAccel(double wheelRadius, double robotWeight, MotorType motor) {
        //unit conversion
        double wheelRadMeters = wheelRadius * 0.0254;
        System.out.println(wheelRadMeters);
        double robotMassKg = robotWeight / 2.205;
        double maxSpeedRad = motor.maxRPM() * ((2.0 * Math.PI) / 60.0);
        double maxTorqueNM = motor.maxTorque() * 0.113;

        //max accel in radians
        double maxWheelAccelRad = (4.0 * maxTorqueNM) / (robotMassKg * Math.pow(wheelRadMeters, 2));

        //max accel in motor percentage
        return maxWheelAccelRad / maxSpeedRad;

    }

    private double calculateMaxFrictionAccelRad() {
        double wheelRadMeters = wheelRadius * 0.0254;
        double frictionCoefficient = 0.02;

        return (frictionCoefficient * 9.81) / (wheelRadMeters);
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
