package org.vulcanrobotics.math.geometry;

public class Pose {

    public final double x;
    public final double y;
    public final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Pose old) {
        this.x = old.x;
        this.y = old.y;
        this.heading = old.heading;
    }

    public Pose(Vector vec, double heading) {
        this.x = vec.x;
        this.y = vec.y;
        this.heading = heading;
    }

    public Vector vector() {
        return new Vector(x, y);
    }

    public Vector headingVector() {
        return new Vector(Math.cos(heading), Math.sin(heading));
    }

    public Pose plus(Pose pose) {
        return new Pose(this.x + pose.x, this.y + pose.y, this.heading + pose.heading);
    }

    public Pose minus(Pose pose) {
        return new Pose(this.x - pose.x, this.y - pose.y, this.heading - pose.heading);
    }

    public Pose multiply(double scalar) {
        return new Pose(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose divide(double scalar) {
        return new Pose(this.x / scalar, this.y / scalar, this.heading / scalar);
    }


}
