package org.vulcanrobotics.follower;

import org.apache.commons.math3.linear.MatrixUtils;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.drivetrains.Mecanum;

public class TestFollower extends Follower{

    Mecanum model;

    public TestFollower(Path path, Mecanum model) {
        super(path, model);
        this.model = model;
    }

    public void run() throws Exception {
        Pose robotPose = model.getRobotPose();

        Pose targetPose = new Pose(80.0, 80.0, 0.0);
        double angleToPoint = Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x);
        Pose velocityOut = new Pose(Math.cos(angleToPoint), Math.sin(angleToPoint), 0.0);

        double[] outputWheelVelocities = model.calculateWheelVelocities(MatrixUtils.createColumnRealMatrix(new double[] {velocityOut.x, velocityOut.y, velocityOut.heading}));

        model.update(outputWheelVelocities);
    }

}
