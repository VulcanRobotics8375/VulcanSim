package org.vulcanrobotics.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;

public class Derivative implements UnivariateFunction {
    UnivariateFunction function;
    double h;

    public Derivative(UnivariateFunction function) {
        this.function = function;
        this.h = 0.00001;
    }

    public Derivative(UnivariateFunction function, double h) {
        this.function = function;
        this.h = h;
    }

    @Override
    public double value(double x) {
        return (function.value(x + h) - function.value(x - h)) / (2 * h);
    }
}