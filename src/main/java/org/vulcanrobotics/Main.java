package org.vulcanrobotics;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.SplitPane;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import org.apache.commons.math3.linear.MatrixUtils;
import org.vulcanrobotics.follower.TestFollower;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.math.geometry.Vector;
import org.vulcanrobotics.path.Path;
import org.vulcanrobotics.sim.RobotModel;
import org.vulcanrobotics.sim.drivetrains.Mecanum;
import org.vulcanrobotics.sim.motors.NeverestOrbital20;

import java.io.FileInputStream;

public class Main extends Application {

    public static ImageView robotView;
    int boardDimensions = 800;
    int controlBoardWidth = 400;
    public static RobotModel model;
    public static Runnable followerTask;
    private volatile boolean running = false;


    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("VulcanSim dev build 0");

        var board = new Image(new FileInputStream("src/main/resources/img/ffField.png"));
        var boardView = new ImageView(board);
        boardView.setFitHeight(boardDimensions);
        boardView.setFitWidth(boardDimensions);

        var robot = new Image(new FileInputStream("src/main/resources/img/robot.png"));
        robotView = new ImageView(robot);
        //TODO add scalars for this
        robotView.setFitWidth((800.0/144.0) * 12);
        robotView.setFitHeight((800.0/144.0) * 18);

        Pane robotField = new Pane();
        robotField.getChildren().add(boardView);
        robotField.getChildren().add(robotView);

        VBox controlBox = new VBox();
        Label title = new Label("controls");
        controlBox.getChildren().add(title);
        Button start = new Button("start");
        start.setOnMouseClicked(event -> {
            running = true;
            startRobot();
        });
        Button stop = new Button("stop");
        stop.setOnMouseClicked(event -> {
            running = false;
        controlBox.getChildren().add(start);
        controlBox.getChildren().add(stop);

        SplitPane splitView = new SplitPane();

        splitView.getItems().addAll(robotField, controlBox);
        splitView.setDividerPosition(0, (double)boardDimensions / (boardDimensions + controlBoardWidth));

        stage.setScene(new Scene(splitView, boardDimensions + controlBoardWidth, boardDimensions));
        stage.show();
    }

    private void startRobot() {
        new Thread(() -> {
            while(running) {
                long startTime = System.nanoTime();
                followerTask.run();
                model.update();
                Platform.runLater(() -> setRobotPosition(model.getRobotPose()));
                try {
                    long elapsedTimeNS = System.nanoTime() - startTime;
                    long timeLeft = ((long)(model.getLoopTime() * 1000)) - (elapsedTimeNS / 1000000);
                    Thread.sleep(timeLeft);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
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

        TestFollower follower = new TestFollower(new Path() {
            @Override
            public Vector error(Pose robot) {
               return null;
            }

            @Override
            public Pose get(double t) {
                return null;
            }

            @Override
            public Pose tangentVec(double t) {
                return null;
            }
        });
        followerTask = () -> {
            try {
                follower.run();
            } catch (Exception e) {
                e.printStackTrace();
            }
        };
        model = follower.getModel();
        launch();


    }
}