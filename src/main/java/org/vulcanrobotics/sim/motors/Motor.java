package org.vulcanrobotics.sim.motors;

public abstract class Motor {

    protected double speed;
    private final MotorType motorProperties;

    public Motor(Class<? extends Motor> motorSubClass) {
        motorProperties = motorSubClass.getAnnotation(MotorType.class);
    }

    public void speed(double input) {
        speed = Math.min(Math.max(input, -1.0), 1.0);
    }

    public double getSpeed() {
        return speed;
    }

    public MotorType getMotorProperties() {
        return motorProperties;
    }

}
