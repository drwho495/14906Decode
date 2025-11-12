package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;

public class SubsystemManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private OpModeOptions managerOptions = new OpModeOptions();
    private LinearOpMode opMode;

    private boolean isShooting = false;
    private final boolean hoodServoManual = true;

    private boolean stateStart = true;

    // do NOT add a constructor to any of the subsystems!
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public SubsystemManager(LinearOpMode newOpMode) {
        opMode = newOpMode;
        shooterSubsystem.setLinearTeleop(this.opMode);
        intakeSubsystem.setLinearTeleop(this.opMode);
    }

    public OpModeStates getState() {
        return currentState;
    }

    public void initialiseHardware() {
        shooterSubsystem.initialiseHardware();
        intakeSubsystem.initialiseHardware();
    }

    public void tryPowerOnShooter() {
        shooterSubsystem.powerOn();
    }

    public void tryReverseIntake() {
//        if (!isShooting) {
            intakeSubsystem.setIntakePower(-1);
//        }
    }

    public void tryPowerOffIntake() {
//        if (!isShooting) {
            intakeSubsystem.powerIntakeOff();
//        }
    }

    public void trySetIntakePower(double newPower) {
//        if (!isShooting) {
            intakeSubsystem.setIntakePower(newPower);
//        }
    }

    public boolean isIntakeOn() {
        return intakeSubsystem.isIntakeOn();
    }

    public void tryToggleShooter() {
        shooterSubsystem.toggleShooterPower();
    }

    public void tryPowerOffShooter() {
        shooterSubsystem.powerOff();
    }

    public void tryStartShootElement() {
        isShooting = true;
    }

    public void tryStopShootElement() {
        isShooting = false;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public void setState(OpModeStates newState) {
        if (currentState != newState) {
            stateStart = true;

            currentState = newState;
        }
    }

    public void trySetShooterVelocity(double velocity) {
        shooterSubsystem.setCustomVelocity(velocity);
    }

    public void trySetHoodServoPos(double newPos) {
        if (hoodServoManual) {
            shooterSubsystem.setHoodPos(newPos);
        }
    }

    public void setManagerOptions(OpModeOptions newOptions) {
        managerOptions = newOptions;
    }

    public void update() {
        if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                if (isShooting) {
//                    if (shooterSubsystem.ready()) {
//                        intakeSubsystem.setIntakePower(.5);
//                    }
                    shooterSubsystem.openFinger();
                } else {
                    shooterSubsystem.closeFinger();
                }
                break;

            case PARK:

                break;
        }

        stateStart = false;

        shooterSubsystem.update();
        intakeSubsystem.update();
    }

    public Double[] getShooterVelocities() {
        return shooterSubsystem.getVelocities();
    }
}