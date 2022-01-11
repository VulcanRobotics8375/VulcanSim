package org.vulcanrobotics.sim;

public abstract class RobotModel {

    public double x, y, theta;

    public void updateDeltas(double dx, double dy, double dTheta) {
        x += dx;
        y += dy;
        theta += dTheta;
    }

    public abstract void update(double... powers) throws Exception;

}
