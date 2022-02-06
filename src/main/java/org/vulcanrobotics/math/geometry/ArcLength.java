package org.vulcanrobotics.math.geometry;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.FastMath;

public class ArcLength implements UnivariateFunction {
    PolynomialSplineFunction derivative;
    Integrand integrand;
    SimpsonIntegrator homer;
    double startX;

    public ArcLength(PolynomialSplineFunction function) {
        this.derivative = function.polynomialSplineDerivative();
        this.integrand = new Integrand();
        this.homer = new SimpsonIntegrator();
        this.startX = 0;
    }

    @Override
    public double value(double endX) {
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