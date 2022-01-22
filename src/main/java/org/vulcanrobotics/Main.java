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
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;

import java.io.FileInputStream;

public class Main extends Application {

    public static ImageView robotView;
    int boardDimensions = 800;
    public static RobotModel model;


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

        new Thread(() -> {
            int i = 0;
            model.setRobotPose(new Pose(50.0, 20.0, 0.0));
            while(true) {
                i++;
                    try {
                        if(i < 60) {
                            model.update(1.0, -1.0, -1.0, 1.0);
                        } else if(i < 120) {
                            model.update(-1.0, 1.0, 1.0, -1.0);
                        } else {
                            i = 0;
                        }
                        Thread.sleep(16);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                Platform.runLater(() -> setRobotPosition(model.getRobotPose()));
            }
        }).start();

        Pane main = new Pane();
        main.getChildren().add(boardView);
        main.getChildren().add(robotView);
        stage.setScene(new Scene(main, boardDimensions, boardDimensions));
        stage.show();
    }

    private static void setRobotPosition(Pose pose) {
        robotView.toFront();
        robotView.setY((pose.y) * (800.0 / 144.0));
        robotView.setX((pose.x) * (800.0 / 144.0));
        robotView.setRotate(-Math.toDegrees(pose.heading));
    }

    private Pose robotPositionToPixelPosition(Pose robotPose) {
       double pixelsPerInch = boardDimensions / 144.0;
       return new Pose(robotPose.x * (pixelsPerInch), (robotPose.y * (pixelsPerInch)), robotPose.heading);
    }

    public static void main(String[] args) throws Exception {
        NeverestOrbital20[] motors = new NeverestOrbital20[] {
                new NeverestOrbital20(),
                new NeverestOrbital20(),
                new NeverestOrbital20(),
                new NeverestOrbital20()
        };
        Mecanum mecanum = new Mecanum(4, 4, 1.887, 30, motors);
//        Pose velocityOutTest = mecanum.calculateRobotVelocity(MatrixUtils.createColumnRealMatrix(new double[] {2.0 * Math.PI, 2.0 * Math.PI, 2.0 * Math.PI, 2.0 * Math.PI}));
//        System.out.println(velocityOutTest.toString());
        model = mecanum;
        launch();


    }
}