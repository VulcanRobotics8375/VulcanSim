package org.vulcanrobotics.sim;

import org.vulcanrobotics.math.geometry.Pose;

public abstract class RobotModel {

    protected Pose robotPose = new Pose();
    protected Pose robotPoseVelocity = new Pose();
    protected Pose robotPoseAccel = new Pose();

    public abstract void update(double... powers) throws Exception;



    //getter and setter for robotPose
    public Pose getRobotPose() {
        return robotPose;
    }
    public void setRobotPose(Pose newPose) {
        robotPose = new Pose(newPose);
    }

}
