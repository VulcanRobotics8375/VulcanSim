package org.vulcanrobotics.follower;

import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;

public abstract class Follower {
    protected RobotModel model;

    public Follower(Path path) {
    }

    protected void setModel(RobotModel model) {
        this.model = model;
    }

    public RobotModel getModel() {
        return model;
    }

}
