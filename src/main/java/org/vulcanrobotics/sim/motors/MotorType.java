package org.vulcanrobotics.sim.motors;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface MotorType {

    //identifiers
    String name();
    int id();

    //attributes
    //torque units -- in-lbs
    double maxRPM();
    double maxTorque();
    double backDriveTorque();


}
