package org.vulcanrobotics.math.utils;

import static java.lang.Math.*;

public class Angle {

    public static double normalize(double angle) {
        while(angle < 0) {
            angle += 2.0 * PI;
        }
        while(angle >= 2.0 * PI) {
            angle -= 2.0 * PI;
        }
        return angle;
    }

}
