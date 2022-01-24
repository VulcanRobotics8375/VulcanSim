package org.vulcanrobotics.sim;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.sim.motors.Motor;

public abstract class RobotModel {

    protected volatile Pose robotPose = new Pose();
    protected volatile Pose robotPoseVelocity = new Pose();
    protected double loopTime = 1.0 / 60.0;

    protected double wheelRadius;
    protected double robotWeight;
    protected Motor[] motors;

    public abstract void update(double... powers) throws Exception;


    //getter and setter for robotPose
    public Pose getRobotPose() {
        return robotPose;
    }
    public void setRobotPose(Pose newPose) {
        robotPose = new Pose(newPose);
    }

}
