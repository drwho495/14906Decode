//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode.Base.HardwareBases;

//import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ComplexServo {
    private ServoImplEx servo;
    private double maxAngle;
    private double minAngle;
    private double maxPosition;
    private double minPosition;
    private double positionOffset = 0;
    private double positionMultiplier = 1;
    private double lastPosition = 0;

    public ComplexServo(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        this.maxPosition = 1.0;
        this.minPosition = 0.0;
        this.servo = (ServoImplEx)hw.get(ServoImplEx.class, servoName);
        this.minAngle = this.toRadians(minAngle, angleUnit);
        this.maxAngle = this.toRadians(maxAngle, angleUnit);
    }

    public ComplexServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        this(hw, servoName, minDegree, maxDegree, AngleUnit.DEGREES);
    }

    public void rotateByAngle(double angle, AngleUnit angleUnit) {
        angle += this.getAngle(angleUnit);
        this.turnToAngle(angle, angleUnit, false);
    }

    public void rotateByAngle(double degrees) {
        this.rotateByAngle(degrees, AngleUnit.DEGREES);
    }

    public void turnToAngle(double angle, AngleUnit angleUnit, boolean force) {
        this.setPosition(calculateAngleToServoRawPosition(angle, angleUnit), force);
    }

    public void turnToAngle(double degrees, boolean force) {
        this.turnToAngle(degrees, AngleUnit.DEGREES, force);
    }

    public void turnToAngle(double degrees) {
        this.turnToAngle(degrees, false);
    }

    public void rotateBy(double position) {
        position += this.getPosition();
        this.setPosition(position, false);
    }

    public void setPosition(double position, boolean force) {
        position = Range.clip((positionMultiplier * position) + calculateAngleToServoRawPosition(positionOffset, AngleUnit.RADIANS), 0.0, 1.0);

        if (force || position != lastPosition) {
            servo.setPosition(position);
        }

        lastPosition = position;
    }

    public void setPosition(double position) {
        setPosition(position, false);
    }

    public void setPositionMultiplier(double positionMultiplier) {
        this.positionMultiplier = positionMultiplier;
    }

    public void setPositionOffset(double positionOffset, AngleUnit angleUnit) {
        this.positionOffset = this.toRadians(positionOffset, angleUnit);
    }
    public void setPositionOffset(double positionOffset) {
        setPositionOffset(positionOffset, AngleUnit.DEGREES);
    }


    public void setRange(double min, double max, AngleUnit angleUnit) {
        this.minAngle = this.toRadians(min, angleUnit);
        this.maxAngle = this.toRadians(max, angleUnit);
    }

    public void setRange(double min, double max) {
        this.setRange(min, max, AngleUnit.DEGREES);
    }

    public void setInverted(boolean isInverted) {
        this.servo.setDirection(isInverted ? Direction.REVERSE : Direction.FORWARD);
    }

    public boolean getInverted() {
        return Direction.REVERSE == this.servo.getDirection();
    }

    public double getPosition() {
        return this.servo.getPosition();
    }

    public double getAngle(AngleUnit angleUnit) {
        return this.getPosition() * this.getAngleRange(angleUnit) + this.fromRadians(this.minAngle, angleUnit);
    }

    public double getAngle() {
        return this.getAngle(AngleUnit.DEGREES);
    }

    public double getAngleRange(AngleUnit angleUnit) {
        return this.fromRadians(this.maxAngle - this.minAngle, angleUnit);
    }

    public double getAngleRange() {
        return this.getAngleRange(AngleUnit.DEGREES);
    }

    public void disable() {
        this.servo.setPwmDisable();
    }

    public String getDeviceType() {
        String port = Integer.toString(this.servo.getPortNumber());
        String controller = this.servo.getController().toString();
        return "SimpleServo: " + port + "; " + controller;
    }

    private double toRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toRadians(angle) : angle;
    }

    private double fromRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(angle) : angle;
    }

    private double calculateAngleToServoRawPosition(double fromAngle, AngleUnit angleUnit) {
        double angleRadians = Range.clip(this.toRadians(fromAngle, angleUnit), this.minAngle, this.maxAngle);
        return (angleRadians - this.minAngle) / this.getAngleRange(AngleUnit.RADIANS);
    }
}
