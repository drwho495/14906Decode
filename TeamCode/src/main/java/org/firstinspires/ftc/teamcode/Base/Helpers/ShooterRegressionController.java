package org.firstinspires.ftc.teamcode.Base.Helpers;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.Arrays;

public class ShooterRegressionController {
    private final Spline hoodSpline = new Spline();
    private final Spline rpmSpline = new Spline();
    private final Polygon hoodPolygon = new Polygon();
    private final Polygon rpmPolygon = new Polygon();
    private RegressionMethod regressionMethod = RegressionMethod.SPLINE;

    // we populate these by default so we can edit these values with Panels.
    public ArrayList<Pose> hoodDataPoints = new ArrayList<>(
            Arrays.asList(
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0)
            )
    );
    public ArrayList<Pose> rpmDataPoints = new ArrayList<>(
            Arrays.asList(
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0)
            )
    );

    public void build() {
        if (regressionMethod == RegressionMethod.SPLINE) {
            hoodSpline.populate(hoodDataPoints);
            rpmSpline.populate(rpmDataPoints);

            hoodSpline.buildCurve();
            rpmSpline.buildCurve();
        } else if (regressionMethod == RegressionMethod.POLYGON) {
            hoodPolygon.populate(hoodDataPoints);
            rpmPolygon.populate(rpmDataPoints);
        }
    }

    public void setRegressionMethod(RegressionMethod regressionMethod) {
        this.regressionMethod = regressionMethod;
    }

    public RegressionMethod getRegressionMethod() {
        return regressionMethod;
    }

    public void addHoodPoint(double x, double y) {
        hoodDataPoints.add(new Pose(x, y));
    }

    public void addRPMPoint(double x, double y) {
        rpmDataPoints.add(new Pose(x, y));
    }

    public void addPoint(double x, double hoodPointY, double RPMPointY) {
        addHoodPoint(x, hoodPointY);
        addRPMPoint(x, RPMPointY);
    }

    public double getHoodOutput(double x) {
        if (regressionMethod == RegressionMethod.SPLINE) {
            return hoodSpline.getY(x);
        } else if (regressionMethod == RegressionMethod.POLYGON) {
            return hoodPolygon.getY(x);
        }

        return 0;
    }

    public double getRPMOutput(double x) {
        if (regressionMethod == RegressionMethod.SPLINE) {
            return rpmSpline.getY(x);
        } else if (regressionMethod == RegressionMethod.POLYGON) {
            return rpmPolygon.getY(x);
        }

        return 0;
    }

    public void clear() {
        if (regressionMethod == RegressionMethod.SPLINE) {
            rpmSpline.clear();
            hoodSpline.clear();
        } else if (regressionMethod == RegressionMethod.POLYGON) {
            rpmPolygon.clear();
            hoodPolygon.clear();
        }

        rpmDataPoints.clear();
        hoodDataPoints.clear();
    }
}