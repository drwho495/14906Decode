package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;

public class ShooterSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor leftShooterMotor;
    private ComplexMotor rightShooterMotor;
    private ComplexServo launchingServo;
    private double motorVelo = 2400;
    private double shooterArmPosition = Parameters.SHOOTER_ARM_DOWN;
    private boolean powerOff = true;

//    private double veloP = .001;
//    private double veloI = 0.0;
//    private double veloD = 0;

    private final double velocityMultiplier = 304.0/6000;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    @Override
    public void initialiseHardware() {
        leftShooterMotor = new ComplexMotor("leftShooter", thisOpMode);
        rightShooterMotor = new ComplexMotor("rightShooter", thisOpMode);
        launchingServo = new ComplexServo(thisOpMode.hardwareMap, "launchingServo", 0, 180);
        launchingServo.setInverted(false);

        leftShooterMotor.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        leftShooterMotor.setReversed(true);
        rightShooterMotor.setMode(ComplexMotorModes.USE_VELOCITY_PID);
        rightShooterMotor.setReversed(false);

//        leftShooterMotor.setVelocityPIDFCoefficients(veloP, veloI, veloD, 0);
//        rightShooterMotor.setVelocityPIDFCoefficients(veloP, veloI, veloD, 0);
        leftShooterMotor.useCustomVeloPIDLoop(false);
        rightShooterMotor.useCustomVeloPIDLoop(false);
    }

    public void updatePowerFromPosition(Pose position) {
        motorVelo = Parameters.SHOOTER_DEFAULT_RPM;
    }

    public boolean ready() {
        return leftShooterMotor.atVelocity() && rightShooterMotor.atVelocity();
    }

    public void powerOff() {
        powerOff = true;
    }

    public void powerOn() {
        powerOff = false;
    }

    public void setCustomVelocity(double newVelo) {
        motorVelo = newVelo;
    }

    public void toggleShooterPower() {
        powerOff = !powerOff;
    }

    public void shooterArmUp() {
        shooterArmPosition = Parameters.SHOOTER_ARM_UP;
    }

    public void shooterArmDown() {
        shooterArmPosition = Parameters.SHOOTER_ARM_DOWN;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

//        leftShooterMotor.setVelocityPIDFCoefficients(veloP, veloI, veloD, 0);
//        rightShooterMotor.setVelocityPIDFCoefficients(veloP, veloI, veloD, 0);

        launchingServo.turnToAngle(shooterArmPosition);

        if (powerOff) {
            leftShooterMotor.setVelo(0);
            rightShooterMotor.setVelo(0);
        } else {
            leftShooterMotor.setVelo(motorVelo * velocityMultiplier);
            rightShooterMotor.setVelo(motorVelo * velocityMultiplier);
        }

        leftShooterMotor.update();
        rightShooterMotor.update();
    }
}
