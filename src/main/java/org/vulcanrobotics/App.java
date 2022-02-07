package org.vulcanrobotics;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.effect.BlendMode;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.shape.Line;
import javafx.stage.Stage;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.vulcanrobotics.follower.ContinuousLookAhead;
import org.vulcanrobotics.follower.Follower;
import org.vulcanrobotics.follower.GuidingVectorField;
import org.vulcanrobotics.follower.TestFollower;
import org.vulcanrobotics.math.geometry.Pose;
import org.vulcanrobotics.path.BasicPath;
import org.vulcanrobotics.sim.RobotModel;

import java.io.FileInputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class App extends Application {

    public static ImageView robotView;
    public static Pane robotField = new Pane();
    int boardDimensions = 800;
    int controlBoardWidth = 400;
    private volatile boolean running = false;
    private Pose poseVelocity;
    private final Label poseVelocityLabel = new Label();
    private Pose pose;
    private final Label poseLabel = new Label();
    double robotWidth = 12, robotLength = 16;
    double elapsedTime = 0;
    public static Follower follower;


    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("VulcanSim dev build 1");

        var board = new Image(new FileInputStream("src/main/resources/img/ffField.png"));
        var boardView = new ImageView(board);
        boardView.setFitHeight(boardDimensions);
        boardView.setFitWidth(boardDimensions);

        var robot = new Image(new FileInputStream("src/main/resources/img/robot.png"));
        robotView = new ImageView(robot);
        robotView.setFitWidth((800.0/144.0) * robotWidth);
        robotView.setFitHeight((800.0/144.0) * robotLength);
        //set robot position to default
        setRobotPosition(new Pose(0, 0, 0));

        robotField.getChildren().add(boardView);
        boardView.toBack();
        robotField.getChildren().add(robotView);

        VBox controlBox = new VBox();
        Label title = new Label("controls");
        controlBox.getChildren().add(title);
        Button start = new Button("start");
        start.setOnMouseClicked(event -> {
            if(!running) {
                running = true;
                startRobot();
            }
        });
        Button stop = new Button("stop");
        stop.setOnMouseClicked(event -> {
            running = false;
        });
        controlBox.getChildren().add(start);
        controlBox.getChildren().add(stop);
        controlBox.getChildren().add(poseVelocityLabel);
        controlBox.getChildren().add(poseLabel);

        TabPane tabs = new TabPane();
        tabs.setTabClosingPolicy(TabPane.TabClosingPolicy.UNAVAILABLE);
        tabs.getTabs().add(new Tab("controls", controlBox));

        SplitPane splitView = new SplitPane();

        splitView.getItems().addAll(robotField, tabs);
        splitView.setDividerPosition(0, (double)boardDimensions / (boardDimensions + controlBoardWidth));

        stage.setScene(new Scene(splitView, boardDimensions + controlBoardWidth, boardDimensions));
        stage.setOnCloseRequest(event -> {
            //onClose Function
            running = false;
        });
        stage.show();
    }

    public static void drawFunction(UnivariateFunction f, double min, double max) {
        double i = min;
        ArrayList<Line> lines = new ArrayList<>();
        while(i < max) {

            if(i + 0.2 < max) {
                double startY = f.value(i);
                double endY = f.value(i + 0.2);
                Line line = new Line((i * (800.0 / 144.0)) + 400, (-startY * (800.0 / 144.0)) + 400, ((i + 0.2) * (800.0 / 144.0)) + 400, (-endY * (800.0 / 144.0)) + 400);
                line.setStrokeWidth(5.0);
                line.setOpacity(0.2);
//                line.setBlendMode(BlendMode.OVERLAY);
                lines.add(line);
            }
            i+= 0.2;
        }
        robotField.getChildren().addAll(lines);
    }

    private void startRobot() {
        new Thread(() -> {
            RobotModel model = follower.getModel();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Pose lastPose = model.getRobotPose();
            while(running) {
                long startTime = System.nanoTime();
                follower.run();
                model.update();
                Pose finalLastPose = lastPose;
                Platform.runLater(() -> {
                    setRobotPosition(model.getRobotPose());
                    Line robotLine = new Line((finalLastPose.x * (800.0 / 144.0)) + 400, (-finalLastPose.y * (800.0 / 144.0)) + 400, (model.getRobotPose().x * (800.0 / 144.0)) + 400, (-model.getRobotPose().y * (800.0 / 144.0)) + 400);
                    robotLine.setStroke(Color.BLUE);
                    robotLine.setStrokeWidth(4.0);
                    robotLine.setOpacity(0.3);
                    robotField.getChildren().add(robotLine);
                });
                lastPose = model.getRobotPose();
                try {
                    long elapsedTimeNS = System.nanoTime() - startTime;
                    long timeLeft = ((long)(model.getLoopTime() * 1000)) - (elapsedTimeNS / 1000000);
                    poseVelocity = model.getRobotPoseVelocity();
                    pose = model.getRobotPose();
                    DecimalFormat df = new DecimalFormat("0.00");
                    Platform.runLater(() -> {
                            poseVelocityLabel.setText("pose velocity: " + df.format(poseVelocity.x / model.getLoopTime()) + ", " + df.format(poseVelocity.y / model.getLoopTime()) + ", " + df.format(poseVelocity.heading / model.getLoopTime()));
                            poseLabel.setText("pose: " + df.format(pose.x) + ", " + df.format(pose.y) + ", " + df.format(pose.heading));
                    });
                    if(timeLeft > 0) {
                        Thread.sleep(timeLeft);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    private static void setRobotPosition(Pose pose) {
        robotView.toFront();
        robotView.setY(((-pose.y) * (800.0 / 144.0)) + 400.0 - (robotView.getFitHeight() / 2.0));
        robotView.setX(((pose.x) * (800.0 / 144.0) + 400 - (robotView.getFitWidth() / 2.0)));
        robotView.setRotate(-Math.toDegrees(pose.heading) + 90);
    }

    private Pose robotPositionToPixelPosition(Pose robotPose) {
       double pixelsPerInch = boardDimensions / 144.0;
       return new Pose(robotPose.x * (pixelsPerInch), (robotPose.y * (pixelsPerInch)), robotPose.heading);
    }

    public static void main(String[] args) throws Exception {
        ArrayList<Pose> pathPoints = new ArrayList<>();
        pathPoints.add(new Pose(0.0, 10.0, 0.0));
        pathPoints.add(new Pose(17.0, -12.0, 0.0));
        pathPoints.add(new Pose(40.0, -3.0, 0.0));
        pathPoints.add(new Pose(70.0, 0.0, 0.0));


        BasicPath path = new BasicPath(pathPoints);
        follower = new ContinuousLookAhead(path, pathPoints);

        launch();


    }
}