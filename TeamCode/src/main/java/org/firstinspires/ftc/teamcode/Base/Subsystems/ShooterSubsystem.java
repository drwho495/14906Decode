package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterPFState;

@Config
@Configurable
public class ShooterSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor shooterMotor1;
    private ComplexMotor shooterMotor2;
    private ComplexServo fingerServo;
    private ComplexServo hoodServo;
    private ComplexServo turretServoLeft;
    private ComplexServo turretServoRight;
    private VoltageSensor vSensor;

    private boolean hoodCompensationEnabled = false;
    private boolean poweredOff = true;
    private double motorVelo = Parameters.SHOOTER_DEFAULT_RPM;
    private double fingerServoPos = Parameters.FINGER_SERVO_OPEN;
    private double hoodServoPos = Parameters.HOOD_SERVO_DOWN;
    private double shooter1Current = 0;
    private double shooter2Current = 0;
    private double hoodCompensationMultiplier = 9;
    private double lastVelocity = 0;
    private double turretTargetPosition = 0;
    private double servoTargetBacklashOffset = 0;

    private ShooterPFState pfState = ShooterPFState.WANDERING_LOOP;
    public static double wanderingShooterP;
    public static double wanderingShooterF;
    public static double transferringShooterP;
    public static double transferringShooterF;
    public static double fastTransferringShooterP;
    public static double fastTransferringShooterF;

    private final double velocityMultiplier = 304.0 / 6000;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    public void setPFState(ShooterPFState newState) {
        pfState = newState;
        updatePF();
    }

    public ShooterSubsystem() {
        wanderingShooterP = 0.02;
        wanderingShooterF = 0.003;
        transferringShooterP = wanderingShooterP;
        transferringShooterF = wanderingShooterF;
        fastTransferringShooterP = wanderingShooterP;
        fastTransferringShooterF = wanderingShooterF;
    }

    private void updatePF() {
        if (pfState == ShooterPFState.WANDERING_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(wanderingShooterP, 0, 0, wanderingShooterF);
        } else if (pfState == ShooterPFState.TRANSFER_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(transferringShooterP, 0, 0, transferringShooterF);
        } else if (pfState == ShooterPFState.FAST_TRANSFER_LOOP) {
            shooterMotor1.setVelocityPIDFCoefficients(fastTransferringShooterP, 0, 0, fastTransferringShooterF);
        }
    }

    @Override
    public void initialiseHardware() {
        vSensor = thisOpMode.hardwareMap.voltageSensor.iterator().next();

        shooterMotor1 = new ComplexMotor(
                "shooterMotor1",
                thisOpMode,
                vSensor
        );
        shooterMotor2 = new ComplexMotor(
                "shooterMotor2",
                thisOpMode,
                vSensor
        );

        shooterMotor1.enableBrake();
        shooterMotor1.setEncoderState(true);
        shooterMotor1.resetEncoder();
        shooterMotor1.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor1.setReversed(true);

        shooterMotor2.enableBrake();
        shooterMotor2.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        shooterMotor2.setReversed(true);

        shooterMotor1.useCustomVeloPIDLoop(true);
        shooterMotor1.setLinkedMotor(shooterMotor2);

        updatePF();

        fingerServo = new ComplexServo(
                thisOpMode.hardwareMap,
                "fingerServo",
                0,
                180,
                AngleUnit.DEGREES
        );
        fingerServo.setInverted(false);

        hoodServo = new ComplexServo(
                thisOpMode.hardwareMap,
                "hoodServo",
                0,
                180,
                AngleUnit.DEGREES
        );
        hoodServo.setPositionOffset(Parameters.HOOD_SERVO_POSITION_OFFSET, AngleUnit.DEGREES);
        hoodServo.setInverted(false);

        turretServoLeft = new ComplexServo(
                thisOpMode.hardwareMap,
                "turretServoLeft",
                0,
                320,
                AngleUnit.DEGREES
        );
        turretServoLeft.setPositionMultiplier(1);
        turretServoLeft.setPositionOffset(Parameters.TURRET_SERVO_LEFT_ZERO_OFFSET, AngleUnit.RADIANS);
        turretServoLeft.setInverted(false);

        turretServoRight = new ComplexServo(
                thisOpMode.hardwareMap,
                "turretServoRight",
                0,
                320,
                AngleUnit.DEGREES
        );
        turretServoRight.setPositionMultiplier(1);
        turretServoRight.setPositionOffset(Parameters.TURRET_SERVO_RIGHT_ZERO_OFFSET, AngleUnit.RADIANS);
        turretServoRight.setInverted(false);
    }

    public boolean ready() {
        return !poweredOff && shooterMotor1.atVelocity(Parameters.SHOOTER_READY_TOLERANCE * velocityMultiplier);
    }

    public Double[] getVelocities() {
        return new Double[]{shooterMotor1.getVelocity() / velocityMultiplier, shooterMotor2.getVelocity() / velocityMultiplier};
    }

    public void setTurretBacklashOffset(double offset) {
        servoTargetBacklashOffset = offset;
    }

    public void clearTurretBacklashOffset() {
        setTurretBacklashOffset(0);
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
        poweredOff = true;
    }

    public void powerOn() {
        poweredOff = false;
    }

    public void setVelocity(double newVelo) {
        motorVelo = newVelo;
    }

    public double getTargetVelocity() {
        return motorVelo;
    }

    public void toggleShooterPower() {
        poweredOff = !poweredOff;
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

    public boolean isPoweredOn() {
        return !poweredOff;
    }

    public double getHoodAngle() {
        return hoodServoPos;
    }

    public void setTurretPosition(double targetPosition) {
        turretTargetPosition = MathFunctions.normalizeAngle(targetPosition);
    }

    public double getTurretTargetPosition() {
        return turretTargetPosition;
    }

    public double getReachableTurretTargetPosition() {
        return Range.clip(turretTargetPosition, Parameters.TURRET_DEADZONE_ANGLE_FROM_ZERO, ((2 * Math.PI) - Parameters.TURRET_DEADZONE_ANGLE_FROM_ZERO));
    }

    public static boolean angleInTurretRange(double angle) {
        return Parameters.TURRET_DEADZONE_ANGLE_FROM_ZERO <= angle && ((2 * Math.PI) - Parameters.TURRET_DEADZONE_ANGLE_FROM_ZERO) >= angle;
    }

    public boolean turretCanReachTarget() {
        return angleInTurretRange(turretTargetPosition);
    }

    @Override
    public void update() {
        updatePF();

        double hoodServoOffset = 0;

        if (hoodCompensationEnabled) {
            shooter1Current = shooterMotor1.getCurrent();

            if (shooter1Current > 1) {
                hoodServoOffset = Range.clip((shooter1Current) * hoodCompensationMultiplier, 0, 1000);
            }
        }

        if (poweredOff) {
            shooterMotor1.setVelocity(0);
            shooterMotor2.setVelocity(0);
        } else {
            shooterMotor1.setVelocity(motorVelo * velocityMultiplier);
            shooterMotor2.setVelocity(motorVelo * velocityMultiplier);
        }

        double reachableTurretTargetPosition = getReachableTurretTargetPosition() * Parameters.TURRET_ANGLE_MULTIPLIER;
        double backlashOffsetCorrected = ((servoTargetBacklashOffset / 2) * Parameters.TURRET_ANGLE_MULTIPLIER);

        turretServoLeft.setPositionOffset(Parameters.TURRET_SERVO_LEFT_ZERO_OFFSET, AngleUnit.RADIANS);
        turretServoRight.setPositionOffset(Parameters.TURRET_SERVO_RIGHT_ZERO_OFFSET, AngleUnit.RADIANS);
        hoodServo.setPositionOffset(Parameters.HOOD_SERVO_POSITION_OFFSET, AngleUnit.DEGREES);

        turretServoRight.turnToAngle(reachableTurretTargetPosition + backlashOffsetCorrected, AngleUnit.RADIANS, true);
        turretServoLeft.turnToAngle(reachableTurretTargetPosition - backlashOffsetCorrected, AngleUnit.RADIANS, true);

        fingerServo.turnToAngle(fingerServoPos);
        hoodServo.turnToAngle(Range.clip(hoodServoPos - hoodServoOffset, Parameters.HOOD_SERVO_DOWN, Parameters.HOOD_SERVO_UP), AngleUnit.DEGREES, true);

        shooterMotor1.update();
        shooterMotor2.update();

        lastVelocity = getVelocities()[0];
    }
}
