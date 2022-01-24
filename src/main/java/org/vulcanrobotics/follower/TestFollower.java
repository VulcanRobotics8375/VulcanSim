package org.vulcanrobotics.follower;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;

public class TestFollower extends Follower{

    public TestFollower(Path path, RobotModel model) {
        super(path, model);
    }

    public void run() throws Exception {
        Pose robotPose = model.getRobotPose();
        model.update(1.0, 1.0, 1.0, 1.0);
    }

}
