package org.vulcanrobotics.path;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;

import java.util.List;

public class BasicPath extends Path {

    private List<Pose> points;

    public BasicPath(List<Pose> points) {
        this.points = points;
    }

    @Override
    public Vector error(Pose robot) {
        for(int i = 0; i < points.size() - 1; i++) {
            Pose p1 = points.get(i);
            Pose p2 = points.get(i + 1);
            double min, max;
            if(p1.x == p2.x) {
                min = Math.min(p1.y, p2.y);
                max = Math.max(p1.y, p2.y);
            } else {
                min = Math.min(p1.x, p2.x);
                max = Math.max(p1.x, p2.x);
            }

            if(robot.x > min && robot.x < max) {
                Vector p1Vec = p1.vector();
                Vector p2Vec = p2.vector();
                Vector distance = robot.vector().minus(p1Vec);
                Vector pathVec = p2Vec.minus(p1Vec);
                // perpendicular component of the distance from the starting point projected onto the path vector
                return distance.minus(distance.projectOnto(pathVec));
            }
        }
        return null;
    }

    @Override
    public Pose get(double t) {
        return null;
    }

    @Override
    public Pose tangentVec(double t) {
        return null;
    }

}
