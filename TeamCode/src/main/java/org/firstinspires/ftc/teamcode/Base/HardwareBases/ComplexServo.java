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
    private final double maxPosition;
    private final double minPosition;

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
        this.turnToAngle(angle, angleUnit);
    }

    public void rotateByAngle(double degrees) {
        this.rotateByAngle(degrees, AngleUnit.DEGREES);
    }

    public void turnToAngle(double angle, AngleUnit angleUnit) {
        double angleRadians = Range.clip(this.toRadians(angle, angleUnit), this.minAngle, this.maxAngle);
        this.setPosition((angleRadians - this.minAngle) / this.getAngleRange(AngleUnit.RADIANS));
    }

    public void turnToAngle(double degrees) {
        this.turnToAngle(degrees, AngleUnit.DEGREES);
    }

    public void rotateBy(double position) {
        position += this.getPosition();
        this.setPosition(position);
    }

    public void setPosition(double position) {
        this.servo.setPosition(Range.clip(position, 0.0, 1.0));
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
}
