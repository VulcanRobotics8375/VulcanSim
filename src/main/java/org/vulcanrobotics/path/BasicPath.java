package org.vulcanrobotics.path;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;

import java.util.ArrayList;
import java.util.List;

public class BasicPath extends Path {

    private final List<Pose> points;
    private final double pathLen;
    private final List<Double> segmentLengths;

    public BasicPath(List<Pose> points) {
        this.points = points;
        pathLen = calculatePathLength();
        segmentLengths = calculateSegmentLengths();
    }

    /**
     * since the derivative of linear paths usually aren't continuous, we use a bounding box
     * algorithm to find which two points the robot is between, and then use simple vector operations
     * to find the cross track error between those two points.
     *
     */
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
                //this return statement stops the loop once it finds one solution, so it should work at endpoints still
                return distance.minus(distance.projectOnto(pathVec));
            }
        }
        return null;
    }

    //TODO figure this out for basic path
    @Override
    public Pose get(double t) {
        double totalLength = pathLen * t;
        double integrator = 0;
        double x;
        int index;
        for (int i = 0; i < segmentLengths.size(); i++) {
            //this doesnt work lmfao
           if(integrator + segmentLengths.get(i) > totalLength) {
               double m = (totalLength - integrator) / segmentLengths.get(i);
               Pose p1 = points.get(i);
               Pose p2 = points.get(i+1);
               x = p1.x + (m*(p2.x - p1.x));
               break;
           }
           integrator += segmentLengths.get(i);
           x = points.get(i + 1).x;
        }

        return null;

    }

    @Override
    public Pose tangentVec(double t) {
        return null;
    }

    // the length of a linear path is just the pythagorean distance between each segment added together
    private double calculatePathLength() {
        double integrator = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            integrator += Math.hypot(points.get(i + 1).x - points.get(i).x, points.get(i + 1).y - points.get(i).y);
        }
        return integrator;
    }

    private List<Double> calculateSegmentLengths() {
        List<Double> segmentLengths = new ArrayList<>();

        for (int i = 0; i < points.size() - 1; i++) {
            Pose p1 = points.get(i);
            Pose p2 = points.get(i + 1);
            double length = Math.hypot(p2.x - p1.x, p2.y - p1.y);
            segmentLengths.add(length);
        }

        return segmentLengths;
    }

}
