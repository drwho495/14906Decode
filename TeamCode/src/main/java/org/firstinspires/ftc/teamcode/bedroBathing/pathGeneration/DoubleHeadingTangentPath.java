package org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration;

import java.util.ArrayList;

public class DoubleHeadingTangentPath extends BezierCurve {
    private Point startPoint, p1, p2, endPoint;
    private ArrayList<Point> controlPoints = new ArrayList<>();
    private double length = 0;
    private boolean reversed = false;

//    public DoubleHeadingTangentPath(Point start, Point end, double startHeading, double endHeading, double tangentLength) {
//        this.startPoint = start;
//        this.endPoint = end;
//
//        Point startTangent = new Point(Math.cos(startHeading), Math.sin(startHeading)).scale(tangentLength);
//        Point endTangent = new Point(Math.cos(endHeading), Math.sin(endHeading)).scale(tangentLength);
//
//        this.p1 = MathFunctions.addPoints(start, startTangent);
//
//        this.p2 = MathFunctions.addPoints(end, endTangent.scale(-1));
//    }

    /**
     * Create a Double Heading Tangent path.
     *
     * @param start         The start point.
     * @param end           The end point.
     */
    public DoubleHeadingTangentPath(Point start, Point end) {
        this.startPoint = start;
        this.endPoint = end;

        controlPoints.add(this.startPoint);
        controlPoints.add(this.endPoint);

        setTangentAttributes(0, 0, 18);
    }

    /**
     * Set target attributes of current path.
     * Should never be accessed by the user, only by the follower.
     *
     * @param startHeading         The start heading.
     * @param endHeading           The end heading.
     * @param tangentLength        The tangent length.
     */
    public void setTangentAttributes(double startHeading, double endHeading, double tangentLength) {
        double deltaX = this.endPoint.getX() -  this.startPoint.getX();
        double deltaY = this.endPoint.getY() -  this.startPoint.getY();

        reversed = Math.signum(deltaX) == -1 || Math.signum(deltaY) == -1;

        if(reversed) {
            tangentLength *= -1;
        }

        Point startTangent = new Point(Math.cos(startHeading), Math.sin(startHeading)).scale(tangentLength);
        Point endTangent = new Point(Math.cos(endHeading), Math.sin(endHeading)).scale(tangentLength);

        this.p1 = MathFunctions.addPoints(this.startPoint, startTangent);
        this.p2 = MathFunctions.addPoints(this.endPoint, endTangent.scale(-1));

        length = approximateLength();
        super.initializeDashboardDrawingPoints();
    }

    @Override
    public double approximateLength() {
        return Math.sqrt(Math.pow(startPoint.getX() - endPoint.getX(), 2) + Math.pow(startPoint.getY() - endPoint.getY(), 2));
    }

    @Override
    public String pathType() {
        return "DoubleHeadingTangentPath";
    }

    @Override
    public Point getPoint(double t) {
        double u = 1 - t;

        Point returnPoint = MathFunctions.addPoints(startPoint.scale(u * u * u), p1.scale(3 * u * u * t));
        returnPoint = MathFunctions.addPoints(returnPoint, p2.scale(3 * u * t * t));
        returnPoint = MathFunctions.addPoints(returnPoint, endPoint.scale(t * t * t));

        return returnPoint;
    }

    public double getHeading(double t) {
        double u = 1 - t;

        double dx = 3 * u * u * (p1.getX() - startPoint.getX())
                + 6 * u * t * (p2.getX() - p1.getX())
                + 3 * t * t * (endPoint.getX() - p2.getX());

        double dy = 3 * u * u * (p1.getY() - startPoint.getY())
                + 6 * u * t * (p2.getY() - p1.getY())
                + 3 * t * t * (endPoint.getY() - p2.getY());

        if(reversed) {
            return Math.atan2(dy, dx) - Math.PI;
        } else {
            return Math.atan2(dy, dx);
        }
    }

    @Override
    public Vector getDerivative(double t) {
        Vector returnVector = new Vector();

        returnVector.setOrthogonalComponents(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());

        return returnVector;
    }

    @Override
    public Vector getApproxSecondDerivative(double t) {
        return new Vector();
    }

    @Override
    public double getCurvature(double t) {
//        t = MathFunctions.clamp(t, 0, 1);
//        Vector derivative = getDerivative(t);
//        Vector secondDerivative = getSecondDerivative(t);
//
//        if (derivative.getMagnitude() == 0) return 0;
//        return (MathFunctions.crossProduct(derivative, secondDerivative))/Math.pow(derivative.getMagnitude(),3);
        return 0.0; // This class works more like a line than a curve
    }

    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }

    @Override
    public Vector getEndTangent() {
        return MathFunctions.copyVector(MathFunctions.normalizeVector(getDerivative(1)));
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public ArrayList<Point> getControlPoints() {
        return controlPoints;
    }

    @Override
    public Point getFirstControlPoint() {
        return controlPoints.get(0);
    }

    @Override
    public Point getSecondControlPoint() {
        return controlPoints.get(1);
    }

    @Override
    public Point getSecondToLastControlPoint() {
        return controlPoints.get(controlPoints.size()-2);
    }

    @Override
    public Point getLastControlPoint() {
        return controlPoints.get(controlPoints.size()-1);
    }

//    public static void main(String[] args) {
//        Point start = new Point(0, 0);
//        Point end = new Point(15, 10);
//        double startHeading = Math.toRadians(0);    // facing right
//        double endHeading = Math.toRadians(0);     // facing up
//        double tangentLength = 3;
//
//        DoubleHeadingTangentPath curve = new DoubleHeadingTangentPath(start, end, startHeading, endHeading, tangentLength);
//
//        // Print points along the curve
//        for (double t = 0; t <= 1.0; t += 0.1) {
//            Point pt = curve.getPoint(t);
//            double heading = Math.toDegrees(curve.getHeading(t));
//            System.out.printf("t=%.1f: Point %s, Heading %.1f degrees%n", t, pt, heading);
//        }
//    }
}
