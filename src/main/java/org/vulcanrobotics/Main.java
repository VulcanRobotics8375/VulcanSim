package org.vulcanrobotics;

import javafx.application.Application;
import javafx.stage.Stage;
import org.apache.commons.math3.linear.MatrixUtils;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.sim.drivetrains.Mecanum;

public class Main extends Application {
    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("Hello!");
        stage.show();
    }

    public static void main(String[] args) {
        System.out.println(System.getProperty("os.name"));
        Mecanum mecanum = new Mecanum(4, 4, 1);
        Pose output = mecanum.calculateRobotVelocity(MatrixUtils.createColumnRealMatrix(new double[] {1, -1, 1, -1}));
        System.out.println(output.x + ", " + output.y + ", " + output.heading);
        launch();
    }
}