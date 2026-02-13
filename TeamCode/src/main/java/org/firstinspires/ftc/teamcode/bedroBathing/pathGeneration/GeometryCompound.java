package org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration;

import java.util.ArrayList;
import java.util.Arrays;

public class GeometryCompound extends BezierCurve {
    private ArrayList<BezierCurve> compoundPaths;
    private double length = -1;

    public GeometryCompound(BezierCurve... paths) {
        super();
        this.compoundPaths = new ArrayList<>(Arrays.asList(paths));
    }

    public void addPathGeometry(BezierCurve path) {
        compoundPaths.add(path);

        length = -1;
    }

    public void removePathGeometry(int index) {
        compoundPaths.remove(index < 0 ? compoundPaths.size() + index : index);

        length = -1;
    }

    public void removePathGeometry(BezierCurve path) {
        compoundPaths.remove(path);

        length = -1;
    }

    /**
     * This returns the unit tangent Vector at the end of the GeometryCompound.
     *
     * @return returns the tangent Vector.
     */
    @Override
    public Vector getEndTangent() {
        if (!compoundPaths.isEmpty()) {
            return compoundPaths.get(compoundPaths.size() - 1).getEndTangent();
        } else {
            return new Vector();
        }
    }

    /**
     * This gets the length of the GeometryCompound.
     *
     * @return returns the length of the GeometryCompound.
     */
    @Override
    public double approximateLength() {
        if (length == -1) {
            length = 0;

            for (BezierCurve path : compoundPaths) {
                length += path.approximateLength();
            }
        }

        return length;
    }

    /**
     * This returns the Point on the Bezier line that is specified by the parametric t value.
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the Point requested.
     */
    @Override
    public Point getPoint(double t) {
        t = MathFunctions.clamp(t, 0, 1);

        if (!compoundPaths.isEmpty()) {
            double mappedT = (compoundPaths.size() * t);

            for (int i = 0; i < compoundPaths.size(); i++) {
                BezierCurve path = compoundPaths.get(i);

                if ((mappedT - i) >= 0 && (mappedT - i) <= 1) {
                    return path.getPoint(t);
                }
            }
        }

        return new Point(0, 0);
    }

    /**
     * This returns the curvature of the GeometryCompound, which is zero.
     *
     * @param t the parametric t value.
     * @return returns the curvature.
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the GeometryCompound as a Vector, which is a constant slope.
     * The t value doesn't really do anything, but it's there so I can override methods.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested.
     */
    @Override
    public Vector getDerivative(double t) {
        Vector returnVector = new Vector();

        if (!compoundPaths.isEmpty()) {
            returnVector.setOrthogonalComponents(
                    compoundPaths.get(compoundPaths.size() - 1).getPoint(1).getX() - compoundPaths.get(0).getPoint(0).getX(),
                    compoundPaths.get(compoundPaths.size() - 1).getPoint(1).getY() - compoundPaths.get(0).getPoint(0).getY()
            );
        }

        return returnVector;
    }

    /**
     * This returns the second derivative on the Bezier line, which is a zero Vector.
     * Once again, the t is only there for the override.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * This returns the zero Vector, but it's here so I can override the method in the BezierCurve
     * class.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative, which is the zero Vector.
     */
    @Override
    public Vector getApproxSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * Returns the ArrayList of control points for this GeometryCompound.
     *
     * @return This returns the control points.
     */
    @Override
    public ArrayList<Point> getControlPoints() {
        ArrayList<Point> returnList = new ArrayList<>();

        for (BezierCurve path : compoundPaths) {
            returnList.addAll(path.getControlPoints());
        }

        return returnList;
    }

    /**
     * Returns the first control point for this GeometryCompound.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getFirstControlPoint() {
        if (!compoundPaths.isEmpty()) {
            return compoundPaths.get(0).getFirstControlPoint();
        } else {
            return new Point(0,0);
        }
    }

    /**
     * Returns the second control point, or the one after the start, for this GeometryCompound.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getSecondControlPoint() {
        return getLastControlPoint();
    }

    /**
     * Returns the second to last control point for this GeometryCompound.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getSecondToLastControlPoint() {
        return getFirstControlPoint();
    }

    /**
     * Returns the last control point for this GeometryCompound.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getLastControlPoint() {
        if (!compoundPaths.isEmpty()) {
            return compoundPaths.get(1).getLastControlPoint();
        } else {
            return new Point(0,0);
        }
    }

    /**
     * Returns the length of this GeometryCompound.
     *
     * @return This returns the length.
     */
    @Override
    public double length() {
        return this.approximateLength();
    }

    /**
     * Returns the conversion factor of one unit of distance into t value.
     *
     * @return returns the conversion factor.
     */
    @Override
    public double UNIT_TO_TIME() {
        return 1 / approximateLength();
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    @Override
    public String pathType() {
        return "line";
    }
}