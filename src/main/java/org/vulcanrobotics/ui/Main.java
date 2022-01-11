package org.vulcanrobotics.ui;

import javafx.application.Application;
import javafx.stage.Stage;

public class Main extends Application {
    @Override
    public void start(Stage stage) throws Exception {
        stage.setTitle("Hello!");
        stage.show();
    }

    public static void main(String[] args) {
        System.out.println(System.getProperty("os.name"));
        launch();
    }
}