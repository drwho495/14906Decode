package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;

public class ShooterSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor shooterMotor1;
    private ComplexMotor shooterMotor2;
    private ComplexServo fingerServo;
    private ComplexServo hoodServo;

    private double motorVelo = Parameters.SHOOTER_DEFAULT_RPM;
    private boolean powerOff = true;
    private double fingerServoPos = Parameters.FINGER_SERVO_OPEN;
    private double hoodServoPos = Parameters.HOOD_SERVO_DOWN;

    private final double velocityMultiplier = 304.0/6000;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    @Override
    public void initialiseHardware() {
        shooterMotor1 = new ComplexMotor("shooterMotor1", thisOpMode);
        shooterMotor2 = new ComplexMotor("shooterMotor2", thisOpMode);

        shooterMotor1.enableFloat();
        shooterMotor1.setEncoderState(true);
        shooterMotor1.resetEncoder();
        shooterMotor1.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor1.setReversed(true);

        shooterMotor2.enableFloat();
        shooterMotor2.setEncoderState(true);
        shooterMotor2.resetEncoder();
        shooterMotor2.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor2.setReversed(false);

        shooterMotor1.useCustomVeloPIDLoop(false);
        shooterMotor2.useCustomVeloPIDLoop(false);

        shooterMotor1.setVelocityPIDFCoefficients(8, 2.8, 0, 0);
        shooterMotor2.setVelocityPIDFCoefficients(8, 2.8, 0, 0);

        fingerServo = new ComplexServo(thisOpMode.hardwareMap, "fingerServo", 0, 180, AngleUnit.DEGREES);
        fingerServo.setInverted(true);

        hoodServo = new ComplexServo(thisOpMode.hardwareMap, "hoodServo", 0, 180, AngleUnit.DEGREES);
        hoodServo.setInverted(true);
    }

    public void updatePowerFromPosition(Pose position) {
        motorVelo = Parameters.SHOOTER_DEFAULT_RPM;
    }

    public boolean ready() {
        return shooterMotor1.atVelocity();
    }

    public Double[] getVelocities() {
        return new Double[]{shooterMotor1.getVelocity() / velocityMultiplier, shooterMotor2.getVelocity() / velocityMultiplier};
    }

    public void openFinger() {
        fingerServoPos = Parameters.FINGER_SERVO_OPEN;
    }

    public void closeFinger() {
        fingerServoPos = Parameters.FINGER_SERVO_CLOSED;
    }

    public void setHoodPos(double setPos) {
        hoodServoPos = Range.clip(setPos, Parameters.HOOD_SERVO_DOWN, Parameters.HOOD_SERVO_UP);
    }

    public void powerOff() {
        powerOff = true;
    }

    public void powerOn() {
        powerOff = false;
    }

    public void setVelocity(double newVelo) {
        motorVelo = newVelo;
    }

    public double getTargetVelocity() {
        return motorVelo;
    }

    public void toggleShooterPower() {
        powerOff = !powerOff;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        if (powerOff) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setVelocity(0);
        } else {
            shooterMotor1.setVelocity(motorVelo * velocityMultiplier);
            shooterMotor2.setVelocity(motorVelo * velocityMultiplier);
        }

        fingerServo.turnToAngle(fingerServoPos);
        hoodServo.turnToAngle(hoodServoPos);

        shooterMotor1.update();
        shooterMotor2.update();
    }

    public boolean isPoweredOn() {
        return !powerOff;
    }

    public double getHoodAngle() {
        return hoodServoPos;
    }
}
