package org.vulcanrobotics.sim.motors;

public abstract class Motor {

    protected double speed;
    //power differs from speed as it is not the current speed of the motor, it is just what the system thinks it should be going.
    protected double power;
    private final MotorType motorProperties;
    private ZeroPowerBehavior zeroPowerBehavior;

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

    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public void setPower(double input) {
        power = Math.min(Math.max(input, -1.0), 1.0);
    }

    public double getPower() {
        return power;
    }

}
