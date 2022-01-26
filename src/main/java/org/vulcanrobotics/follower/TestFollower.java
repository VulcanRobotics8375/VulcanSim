package org.vulcanrobotics.follower;

import org.apache.commons.math3.linear.MatrixUtils;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.utils.Angle;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.Motor;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;
import org.vulcanrobotics.sim.motors.ZeroPowerBehavior;

public class TestFollower extends Follower{

    Mecanum model;
    NeverestOrbital20 fl, fr, bl, br;

    public TestFollower(Path path) {
        super(path);

        // initialize motors and robot model
        fl = new NeverestOrbital20();
        fr = new NeverestOrbital20();
        bl = new NeverestOrbital20();
        br = new NeverestOrbital20();
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); //FLOAT doesnt work yet, things may break if you use it
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        model = new Mecanum(12, 8, 1.887, 30, new Motor[] {fl, fr, bl, br});
        setModel(model);
    }

    public void run() {
        Pose robotPose = model.getRobotPose();

        Pose targetPose = new Pose(0.0, 30, 0);
        double angleToPoint = Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x);
        Pose velocityOut = new Pose(Math.cos(angleToPoint), Math.sin(angleToPoint), Angle.diff(robotPose.heading, targetPose.heading));

        double[] outputWheelVelocities = model.calculateWheelVelocities(MatrixUtils.createColumnRealMatrix(new double[] {velocityOut.x, velocityOut.y, velocityOut.heading}));
        setPowers(outputWheelVelocities);

    }

    void setPowers(double[] powers) {
        fl.setPower(powers[0]);
        fr.setPower(powers[1]);
        bl.setPower(powers[2]);
        br.setPower(powers[3]);

    }

}
