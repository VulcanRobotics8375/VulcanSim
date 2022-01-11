module org.vulcanrobotics.ui {
    requires javafx.controls;
    requires javafx.fxml;
    requires javafx.web;

    requires org.controlsfx.controls;
    requires eu.hansolo.tilesfx;
    requires java.desktop;
    requires commons.math3;

    opens org.vulcanrobotics.ui to javafx.fxml;
    exports org.vulcanrobotics.ui;
}