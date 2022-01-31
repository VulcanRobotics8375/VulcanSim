package org.vulcanrobotics.path;

import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;

public abstract class Path {

    /**
     * Path utilities that work with ALL PATHS
     * Do not edit this section
     */

    public abstract Vector error(Pose robot);

    //for time variant paths, this returns the target position at time t.
    //for time invariant paths, this probs wont be used lmao
    public abstract Pose get(double t);


    /**
     * This section of the Path class is for the user to add any Path features that they might want for their specific path.
     * For example, a straight line path doesn't need any of the fancy derivative stuff that spline paths need,
     * but we still want our Follower to be able to follow any extension of the Path object.
     */

    public Pose tangentVec(double t) {
        return null;
    }

}
