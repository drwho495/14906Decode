package org.firstinspires.ftc.teamcode.Base.Helpers;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.Arrays;

public class ShooterCurve {
    private final PointsCurve hoodCurve = new PointsCurve();
    private final PointsCurve rpmCurve = new PointsCurve();
    private boolean limitDomain = false;

    public ArrayList<Pose> hoodCurvePoints = new ArrayList<>(
            Arrays.asList(
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0),
                    new Pose(0, 0)
            )
    );
    public ArrayList<Pose> rpmCurvePoints = new ArrayList<>(
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
        populateCurveWithPoseList(hoodCurve, hoodCurvePoints);
        populateCurveWithPoseList(rpmCurve, rpmCurvePoints);

        hoodCurve.buildCurve();
        rpmCurve.buildCurve();
    }

    public void enableDomainLimit() {
        limitDomain = true;
    }

    public void disableDomainLimit() {
        limitDomain = false;
    }

    public static void populateCurveWithPoseList(PointsCurve curve, ArrayList<Pose> list) {
        curve.clear();

        for (Pose pose : list) {
            curve.addPoint(pose.getX(), pose.getY());
        }
    }

    public void addHoodCurvePoint(double x, double y) {
        hoodCurvePoints.add(new Pose(x, y));
    }

    public void addRPMCurvePoint(double x, double y) {
        rpmCurvePoints.add(new Pose(x, y));
    }

    public void addPoint(double x, double hoodPointY, double RPMPointY) {
        addHoodCurvePoint(x, hoodPointY);
        addRPMCurvePoint(x, RPMPointY);
    }

    public double getHoodCurveOutput(double x) {
        return getCurveY(hoodCurve, x);
    }

    public double getRPMCurveOutput(double x) {
        return getCurveY(rpmCurve, x);
    }

    private double getCurveY(PointsCurve curve, double x) {
        if (limitDomain) {
            double minX = curve.getMinX();
            double maxX = curve.getMaxX();

            if (x < minX) {
                x = minX;
            } else if (x > maxX) {
                x = maxX;
            }
        }

        return curve.getY(x);
    }

    public void clear() {
        hoodCurve.clear();
        rpmCurve.clear();

        rpmCurvePoints.clear();
        hoodCurvePoints.clear();
    }
}