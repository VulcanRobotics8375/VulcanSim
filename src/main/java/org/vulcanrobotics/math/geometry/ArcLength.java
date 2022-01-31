package org.vulcanrobotics.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.util.FastMath;

public class ArcLength implements UnivariateFunction {
    Derivative derivative;
    Integrand integrand;
    SimpsonIntegrator homer;
    double startX;

    public ArcLength(UnivariateFunction function) {
        this.derivative = new Derivative(function);
        this.integrand = new Integrand();
        this.homer = new SimpsonIntegrator();
    }

    @Override
    public double value(double endX) {
//        int intervals = (int) (FastMath.abs(endX - startX) / 0.5);
        return homer.integrate(10000000, integrand, startX, endX);
    }

    public class Integrand implements UnivariateFunction {
        public double value(double x) {
            return FastMath.sqrt(1 + FastMath.pow(ArcLength.this.derivative.value(x), 2));
        }
    }

    public void updateStartX(double startX) {
        this.startX = startX;
    }
}