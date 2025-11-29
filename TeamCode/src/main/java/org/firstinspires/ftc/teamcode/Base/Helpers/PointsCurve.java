package org.firstinspires.ftc.teamcode.Base.Helpers;

import org.apache.commons.math3.analysis.function.Gaussian;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

public class PointsCurve {
    private final WeightedObservedPoints pointArray = new WeightedObservedPoints();
//    private Gaussian fittedGaussian = null;
    private double[] coefficients = null;

    public void addPoint(double x, double y) {
        pointArray.add(x, y);
    }

    public void buildCurve() {
        final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(3);
        coefficients = fitter.fit(pointArray.toList());

//        fittedGaussian = new Gaussian(coefficients[0], coefficients[1], coefficients[2]);
    }

    public double getY(double x) {
        if (coefficients != null) {
            return coefficients[0] + (coefficients[1] * x) + (coefficients[2] * (x*x)) + (coefficients[3] * (x*x*x));
        }
        return -1;
    }
}
