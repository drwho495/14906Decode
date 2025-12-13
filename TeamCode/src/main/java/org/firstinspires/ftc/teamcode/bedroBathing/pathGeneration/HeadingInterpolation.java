package org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.bedroBathing.util.HeadingInterpolationType;

public class HeadingInterpolation {
    private HeadingInterpolationType type = HeadingInterpolationType.CONSTANT;
    private double startHeading = 0;
    private double endHeading = 0;
    private double startT = 0;
    private double endT = 0;
    private boolean tangentIsReversed = false;
    private Supplier<Double> headingReturnMethod = null;

    public double getHeadingGoalAtPoint(double t, Vector closestPointTangentVector) {
        if (type == HeadingInterpolationType.TANGENTIAL) {
            if (tangentIsReversed)
                return MathFunctions.normalizeAngle(closestPointTangentVector.getTheta() + Math.PI);
            return closestPointTangentVector.getTheta();
        } else {
            if (type == HeadingInterpolationType.VARIABLE && headingReturnMethod != null)
                endHeading = headingReturnMethod.get();

            if (t > endT || type == HeadingInterpolationType.CONSTANT) {
                return MathFunctions.normalizeAngle(endHeading);
            }
            return MathFunctions.normalizeAngle(startHeading + MathFunctions.getTurnDirection(startHeading, endHeading) * MathFunctions.getSmallestAngleDifference(endHeading, startHeading) * ((t - startT) / (endT - startT)));
        }
    }

    public void setVariableEndHeading(double endHeading) {
        this.endHeading = endHeading;
    }

    public void setTangentReversed(boolean set) {
        this.tangentIsReversed = set;
    }

    public boolean tInRange(double t) {
        return t >= startT && t <= endT;
    }

    public double getEndT() {
        return endT;
    }

    public double getStartT() {
        return startT;
    }

    public double getStartHeading() {
        return startHeading;
    }

    public double getEndHeading() {
        return endHeading;
    }

    public boolean isTangentReversed() {
        return tangentIsReversed;
    }

    public HeadingInterpolationType getType() {
        return type;
    }

    public static HeadingInterpolation createConstantHeadingInterpolation(double startT, double endHeading, double endT) {
        HeadingInterpolation newHeadingInterpolation = new HeadingInterpolation();

        newHeadingInterpolation.type = HeadingInterpolationType.CONSTANT;
        newHeadingInterpolation.startHeading = endHeading;
        newHeadingInterpolation.endHeading = endHeading;
        newHeadingInterpolation.startT = startT;
        newHeadingInterpolation.endT = endT;

        return newHeadingInterpolation;
    }

    public static HeadingInterpolation createLinearHeadingInterpolation(double startT, double startHeading, double endHeading, double endT) {
        HeadingInterpolation newHeadingInterpolation = new HeadingInterpolation();

        newHeadingInterpolation.type = HeadingInterpolationType.LINEAR;
        newHeadingInterpolation.startHeading = startHeading;
        newHeadingInterpolation.endHeading = endHeading;
        newHeadingInterpolation.startT = startT;
        newHeadingInterpolation.endT = endT;

        return newHeadingInterpolation;
    }

    public static HeadingInterpolation createTangentHeadingInterpolation(double startT, boolean reversed, double endT) {
        HeadingInterpolation newHeadingInterpolation = new HeadingInterpolation();

        newHeadingInterpolation.type = HeadingInterpolationType.TANGENTIAL;
        newHeadingInterpolation.tangentIsReversed = reversed;
        newHeadingInterpolation.startT = startT;
        newHeadingInterpolation.endT = endT;

        return newHeadingInterpolation;
    }

    public static HeadingInterpolation createVariableHeadingInterpolation(double startT, double startHeading, double endHeading, Supplier<Double> headingReturnMethod, double endT) {
        HeadingInterpolation newHeadingInterpolation = new HeadingInterpolation();

        newHeadingInterpolation.type = HeadingInterpolationType.VARIABLE;
        newHeadingInterpolation.startHeading = startHeading;
        newHeadingInterpolation.endHeading = endHeading;
        newHeadingInterpolation.startT = startT;
        newHeadingInterpolation.endT = endT;
        newHeadingInterpolation.headingReturnMethod = headingReturnMethod;

        return newHeadingInterpolation;
    }
}
