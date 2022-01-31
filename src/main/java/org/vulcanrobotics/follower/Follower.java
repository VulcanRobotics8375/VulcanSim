package org.vulcanrobotics.follower;

import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;

public abstract class Follower {
    protected RobotModel model;
    protected Path path;

    public Follower(Path path) {
        this.path = path;
    }

    public abstract void run();

    protected void setModel(RobotModel model) {
        this.model = model;
    }

    public RobotModel getModel() {
        return model;
    }

}
