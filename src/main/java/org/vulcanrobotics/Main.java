package org.vulcanrobotics;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import org.apache.commons.math3.linear.MatrixUtils;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;

import javax.imageio.ImageIO;
import java.io.FileInputStream;

public class Main extends Application {

    ImageView robotView;
    int boardDimensions = 800;

    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("VulcanSim dev build 0");
        var board = new Image(new FileInputStream("src/main/resources/img/ffField.png"));
        var boardView = new ImageView(board);
        boardView.setFitHeight(boardDimensions);
        boardView.setFitWidth(boardDimensions);

        var robot = new Image(new FileInputStream("src/main/resources/img/robot.png"));
        robotView = new ImageView(robot);
        robotView.setFitWidth((800.0/144.0) * 12);
        robotView.setFitHeight((800.0/144.0) * 18);

//        setRobotPosition(robotPositionToPixelPosition(new Pose(72, 72, 0)));
        robotView.toFront();

        Pane main = new Pane();
        main.getChildren().add(boardView);
        main.getChildren().add(robotView);
        stage.setScene(new Scene(main, boardDimensions, boardDimensions));
        stage.show();
    }

    private void setRobotPosition(Pose pose) {
        robotView.toFront();
        robotView.setY(pose.y - (robotView.getFitHeight() / 2.0));
        robotView.setX(pose.x - (robotView.getFitWidth() / 2.0));
        robotView.setRotate(-Math.toDegrees(pose.heading));
    }

    private Pose robotPositionToPixelPosition(Pose robotPose) {
       double pixelsPerInch = boardDimensions / 144.0;
       return new Pose(robotPose.x * (pixelsPerInch), (robotPose.y * (pixelsPerInch)), robotPose.heading);
    }

    public static void main(String[] args) throws InterruptedException {
        System.out.println(System.getProperty("os.name"));
        Mecanum mecanum = new Mecanum(4, 4, 1);
        Pose output = mecanum.calculateRobotVelocity(MatrixUtils.createColumnRealMatrix(new double[] {1, -1, 1, -1}));
        System.out.println(output.x + ", " + output.y + ", " + output.heading);

        NeverestOrbital20 motor = new NeverestOrbital20();

        launch();
    }
}