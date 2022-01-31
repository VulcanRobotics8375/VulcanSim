package org.vulcanrobotics.follower;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.drivetrains.Mecanum;

public abstract class Follower {
    protected RobotModel model;

    public Follower(Path path, RobotModel model) {
        this.model = model;
    }



}
