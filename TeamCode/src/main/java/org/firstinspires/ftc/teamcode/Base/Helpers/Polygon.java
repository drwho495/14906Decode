package org.firstinspires.ftc.teamcode.Base.Helpers;

import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class Polygon {
    private List<Pose> points = new ArrayList<>();
    private List<Pose> transformedPoints = new ArrayList<>();

    private double offsetX = 0;
    private double offsetY = 0;
    private double offsetRotZ = 0;

    public Polygon(List<Pose> poses) {
        points = poses;
    }

    public Polygon() {
    }

    public static Polygon makeRectangle(double x, double y) {
        Polygon returnPoly = new Polygon();

        returnPoly.addPoint(new Pose(-x/2, y/2));
        returnPoly.addPoint(new Pose(x/2, y/2));
        returnPoly.addPoint(new Pose(x/2, -y/2));
        returnPoly.addPoint(new Pose(-x/2, -y/2));
        returnPoly.addPoint(new Pose(-x/2, y/2));

        return returnPoly;
    }

    public void setOffsets(double x, double y, double rotZ) {
        offsetX = x;
        offsetY = y;
        offsetRotZ = rotZ;

        transformedPoints.clear();
    }

    public void setOffsets(Pose offsetPose) {
        setOffsets(offsetPose.getX(), offsetPose.getY(), Math.toDegrees(offsetPose.getHeading()));
    }

    public void addPoint(Pose pose) {
        points.add(pose);

        transformedPoints.clear();
    }

    public List<Pose> getPoints() {
        return points;
    }

    public List<Pose> getTransformedPoints() {
        if (transformedPoints.size() != points.size() || transformedPoints.isEmpty()) {
            Pose center = getTransformedCenter();
            double angleRadians = Math.toRadians(offsetRotZ);

            transformedPoints.clear();

            // rotate points around center
            double cos = Math.cos(angleRadians);
            double sin = Math.sin(angleRadians);

            for (Pose point : points) {
                double dx = point.getX() - center.getX();
                double dy = point.getY() - center.getY();

                double rx = (dx * cos) - (dy * sin);
                double ry = (dx * sin) + (dy * cos);

                transformedPoints.add(new Pose(rx + center.getX(), ry  + center.getY()));
            }
        }

        return transformedPoints;
    }

    public Pose getCenter() {
        double sumX = 0;
        double sumY = 0;

        for (Pose point : points) {
            sumX += point.getX();
            sumY += point.getY();
        }

        return new Pose(sumX / points.size(), sumY / points.size());
    }

    public Pose getTransformedCenter() {
        return getCenter().add(new Pose(offsetX, offsetY));
    }

    public boolean contains(Pose point) {
        getTransformedPoints(); // make sure `transformedPoints` is set

        if (distanceToWire(point) <= 1e-9) {
            return true;
        }

        int crossings = 0;
        Pose currentVertex, nextVertex;

        for (int i = 0; i < points.size(); i++) {
            if (i + 1 == points.size()) break;

            currentVertex = transformedPoints.get(i);
            nextVertex = transformedPoints.get(i + 1);

            if (((currentVertex.getY() > point.getY()) != (nextVertex.getY() > point.getY())) &&
                    (point.getX() < (nextVertex.getX() - currentVertex.getX()) * (point.getY() - currentVertex.getY()) / (nextVertex.getY() - currentVertex.getY()) + currentVertex.getX()))
            {
                crossings++;
            }
        }

        return (crossings % 2 == 1);
    }

    public boolean contains(Polygon other) {
        List<Pose> otherTransformedPoints = other.getTransformedPoints();

        for (Pose otherPoint : otherTransformedPoints) {
            if (contains(otherPoint))
                return true;
        }

        if (contains(other.getTransformedCenter()))
            return true;

        return false;
    }

    public double distanceToWire(Pose pose) {
        getTransformedPoints(); // make sure `transformedPoints` is set

        double minDistanceSq = Double.MAX_VALUE;
        int numVertices = transformedPoints.size();
        Pose nextVertex;
        Pose currentVertex;

        for (int i = 0; i < numVertices; i++) {
            currentVertex = transformedPoints.get(i);

            if ((i + 1) != numVertices)
                nextVertex = transformedPoints.get(i + 1);
            else
                continue;

            double distance = distancePointToSegment(pose, currentVertex, nextVertex);
            minDistanceSq = Math.min(minDistanceSq, Math.pow(distance, 2));
        }

        return Math.sqrt(minDistanceSq);
    }

    public static double distancePointToSegment(Pose point, Pose segmentA, Pose segmentB) {
        double segmentLengthSq = Math.pow(MathFunctions.distance(segmentA, segmentB), 2);

        if (segmentLengthSq == 0.0) return MathFunctions.distance(point, segmentA);

        double t = ((point.getX() - segmentA.getX())
                * (segmentB.getX() - segmentA.getX())
                + (point.getY() - segmentA.getY())
                * (segmentB.getY()
                - segmentA.getY()))
                / segmentLengthSq;

        Pose closest;
        if (t < 0.0)
            closest = segmentA;
        else if (t > 1.0)
            closest = segmentB;
        else {
            closest = new Pose(
                    segmentA.getX() + t * (segmentB.getX() - segmentA.getX()),
                    segmentA.getY() + t * (segmentB.getY() - segmentA.getY())
            );
        }

        return MathFunctions.distance(point, closest);
    }



}
