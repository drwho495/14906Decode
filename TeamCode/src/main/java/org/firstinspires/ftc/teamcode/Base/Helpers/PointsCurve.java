package org.firstinspires.ftc.teamcode.Base.Helpers;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import java.util.List;

public class PointsCurve {
    private boolean isBuilt = false;
    private double minX = 0;
    private double maxX = 0;
    public final WeightedObservedPoints pointArray = new WeightedObservedPoints();
    public double[] coefficients = null;

    public void addPoint(double x, double y) {
        if (isBuilt()) {
            pointArray.clear();
            isBuilt = false;
        }

        pointArray.add(x, y);
    }

    public void buildCurve() {
        List<WeightedObservedPoint> pointsList = pointArray.toList();

        if (!pointsList.isEmpty()) {
            final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(3);
            coefficients = fitter.fit(pointsList);

            minX = pointsList.get(0).getX();
            maxX = minX;

            for (final WeightedObservedPoint point : pointsList) {
                double pointX = point.getX();

                if (minX > pointX) {
                    minX = pointX;
                } else if (maxX < pointX) {
                    maxX = pointX;
                }
            }

            isBuilt = true;
        }
    }

    public boolean isBuilt() {
        return isBuilt;
    }

    public double getY(double x) {
        if (coefficients != null) {
            return coefficients[0] + (coefficients[1] * x) + (coefficients[2] * (x*x)) + (coefficients[3] * (x*x*x));
        }
        return -1;
    }

    public double getMinX() {
        return minX;
    }

    public double getMaxX() {
        return maxX;
    }

    public void clear() {
        pointArray.clear();
        coefficients = null;
        isBuilt = false;
    }
}
