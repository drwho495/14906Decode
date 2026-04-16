package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;

@Config
public class ShooterSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor shooterMotor1;
    private ComplexMotor shooterMotor2;
    private ComplexServo fingerServo;
    private ComplexServo hoodServo;
    private VoltageSensor vSensor;

    private double motorVelo = Parameters.SHOOTER_DEFAULT_RPM;
    private boolean powerOff = true;
    private double fingerServoPos = Parameters.FINGER_SERVO_OPEN;
    private double hoodServoPos = Parameters.HOOD_SERVO_DOWN;
    private double shooter1Current = 0;
    private double shooter2Current = 0;
    private boolean hoodCompensationEnabled = false;
    private double hoodCompensationMultiplier = 9;
    private double lastVelocity = 0;

    private ShooterPFState pfState = ShooterPFState.WANDERING_LOOP;
    public static double wanderingShooterP = 0.03;
    public static double wanderingShooterF = 0.0032;
    public static double transferringShooterP = 0.032;
    public static double transferringShooterF = 0.00323;
    public static double fastTransferringShooterP = 0.04;
    public static double fastTransferringShooterF = 0.00363;

    private final double velocityMultiplier = 304.0/6000;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    public void setPFState(ShooterPFState newState) {
        pfState = newState;
        updatePF();
    }

    private void updatePF() {
        if (pfState == ShooterPFState.WANDERING_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(wanderingShooterP, 0, 0, wanderingShooterF);
            shooterMotor2.setVelocityPIDFCoefficients(wanderingShooterP, 0, 0, wanderingShooterF);
        } else if (pfState == ShooterPFState.TRANSFER_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(transferringShooterP, 0, 0, transferringShooterF);
            shooterMotor2.setVelocityPIDFCoefficients(transferringShooterP, 0, 0, transferringShooterF);
        } else if (pfState == ShooterPFState.FAST_TRANSFER_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(fastTransferringShooterP, 0, 0, fastTransferringShooterF);
            shooterMotor2.setVelocityPIDFCoefficients(fastTransferringShooterP, 0, 0, fastTransferringShooterF);
        }
    }

    @Override
    public void initialiseHardware() {
        shooterMotor1 = new ComplexMotor("shooterMotor1", thisOpMode);
        shooterMotor2 = new ComplexMotor("shooterMotor2", thisOpMode);

        shooterMotor1.enableBrake();
        shooterMotor1.setEncoderState(true);
        shooterMotor1.resetEncoder();
        shooterMotor1.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor1.setReversed(true);

        shooterMotor2.enableBrake();
        shooterMotor2.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor2.setReversed(false);

        shooterMotor1.useCustomVeloPIDLoop(true);
        shooterMotor1.setLinkedMotor(shooterMotor2);

        updatePF();

        vSensor = thisOpMode.hardwareMap.voltageSensor.iterator().next();

        fingerServo = new ComplexServo(thisOpMode.hardwareMap, "fingerServo", 0, 180, AngleUnit.DEGREES);
        fingerServo.setInverted(true);

        hoodServo = new ComplexServo(thisOpMode.hardwareMap, "hoodServo", 0, 180, AngleUnit.DEGREES);
        hoodServo.setInverted(true);
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

    public void forceCloseFinger() {
        fingerServoPos = Parameters.FINGER_SERVO_CLOSED;
        fingerServo.turnToAngle(fingerServoPos, true);
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

    public void enableHoodCompensation() {
        hoodCompensationEnabled = true;
    }

    public void disableHoodCompensation() {
        hoodCompensationEnabled = false;
    }

    public void setHoodCompensationMultiplier(double newMult) {
        hoodCompensationMultiplier = newMult;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        double hoodServoOffset = 0;

        updatePF();

        if (hoodCompensationEnabled) {
            shooter1Current = shooterMotor1.getCurrent();

            if (shooter1Current > 1) {
                hoodServoOffset = Range.clip((shooter1Current) * hoodCompensationMultiplier, 0, 1000);
            }
        }

        if (powerOff) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setVelocity(0);
        } else {
            shooterMotor1.setVelocity(motorVelo * velocityMultiplier);
            shooterMotor2.setVelocity(motorVelo * velocityMultiplier);
        }

        fingerServo.turnToAngle(fingerServoPos);
        hoodServo.turnToAngle(Range.clip(hoodServoPos - hoodServoOffset, Parameters.HOOD_SERVO_DOWN, Parameters.HOOD_SERVO_UP));

        shooterMotor1.update();
        shooterMotor2.update();

        lastVelocity = getVelocities()[0];
    }

    public boolean isPoweredOn() {
        return !powerOff;
    }

    public double getHoodAngle() {
        return hoodServoPos;
    }
}
