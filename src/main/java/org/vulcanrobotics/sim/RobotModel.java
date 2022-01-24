package org.vulcanrobotics.sim;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.MotorType;
import org.vulcanrobotics.sim.motors.ZeroPowerBehavior;

public abstract class RobotModel {

    protected volatile Pose robotPose = new Pose();
    protected volatile Pose robotPoseVelocity = new Pose();
    protected double loopTime = 1.0 / 60.0;

    protected double wheelRadius;
    protected double robotWeight;

    //this is private now so that all the physics engine stuff cant be overwritten by a subclass
    //might be able to use singleton to instantiate this stuff
    private final Motor[] motors;
    private final MotorType motorProperties;
    private final double maxWheelAccel;

    public RobotModel(double wheelRadius, double robotWeight, Motor[] motors) {
        this.wheelRadius = wheelRadius;
        this.robotWeight = robotWeight;
        this.motors = motors;
        motorProperties = motors[0].getMotorProperties();
        maxWheelAccel = calculateMaxWheelAccel();

    }

    public abstract void update(double... powers) throws Exception;

    //this is where the physics stuff comes in (resistive forces, maximum acceleration, etc)
    protected void setTargetSpeed(int motorId, double targetSpeed) {
        double currentSpeed = motors[motorId].getSpeed();
        double outputSpeed;
        if(targetSpeed != 0 || motors[motorId].getZeroPowerBehavior() == ZeroPowerBehavior.BRAKE) {
            double targetAccel = targetSpeed - currentSpeed;
            double targetOutputAccel = Math.abs(targetAccel) > (maxWheelAccel * loopTime) ? (maxWheelAccel * Math.signum(targetAccel) * loopTime) : targetAccel;
            double resistiveWheelAccel = robotWeight * motorProperties.resistiveCoeff() * -1.0 * currentSpeed;
            outputSpeed = currentSpeed + targetOutputAccel + resistiveWheelAccel;
        } else {
            //TODO calculate force from motor zero power behavior
            //i dont think this implementation works lmfao
            double targetOutputAccel = maxWheelAccel * (motorProperties.backDriveTorque() / motorProperties.maxTorque()) * loopTime * -1.0;
            outputSpeed = currentSpeed + targetOutputAccel;
        }
        motors[motorId].speed(outputSpeed);
    }

    private double calculateMaxWheelAccel() {
        //unit conversion
        double wheelRadMeters = wheelRadius * 0.0254;
        double robotMassKg = robotWeight / 2.205;
        double maxSpeedRad = motorProperties.maxRPM() * ((2.0 * Math.PI) / 60.0);
        double maxTorqueNM = motorProperties.maxTorque() * 0.113;

        //max accel in radians
        double maxWheelAccelRad = (4.0 * maxTorqueNM) / (robotMassKg * Math.pow(wheelRadMeters, 2));

        //max accel in motor percentage
        return maxWheelAccelRad / maxSpeedRad;
    }

    protected double[] getWheelVelocities() {
        double[] motorPowers = new double[motors.length];
        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] = motors[i].getSpeed();
        }
        return motorPowers;
    }

    //getter and setter for robotPose
    public Pose getRobotPose() {
        return robotPose;
    }
    public void setRobotPose(Pose newPose) {
        robotPose = new Pose(newPose);
    }

    protected void setRobotPoseVelocity(Pose velocity) {
        robotPoseVelocity = new Pose(velocity);
    }
}
