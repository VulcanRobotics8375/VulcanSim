module org.vulcanrobotics {
    requires commons.math3;
    requires javafx.controls;
    requires javafx.fxml;
    requires javafx.web;

    requires org.controlsfx.controls;
    requires eu.hansolo.tilesfx;
    requires java.desktop;

    opens org.vulcanrobotics to javafx.fxml;
    exports org.vulcanrobotics;
    exports org.vulcanrobotics.sim;
    exports org.vulcanrobotics.sim.motors;
    exports org.vulcanrobotics.math.geometry;
}