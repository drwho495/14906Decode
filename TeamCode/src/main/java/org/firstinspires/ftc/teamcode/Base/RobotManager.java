package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Helpers.Scheduler;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;

import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private OpModeOptions managerOptions = new OpModeOptions();
    private LinearOpMode opMode;

    private boolean isTransfering = false;
    private boolean startTransfer = true;
    private boolean cancelTransfer = false;

    private boolean isShooting = false;
    private int shootingAmount = -1;
    private boolean shootingStart = false;
    private boolean shootingCancel = false;

    private boolean transferAfterScore = false;

    private boolean driverCanControlClaw = true;
    private Scheduler transferScheduler = new Scheduler();
    private boolean stateStart = true;

    // do NOT add a constructor to any of the subsystems!
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotManager(LinearOpMode newOpMode) {
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

    /**
     * This method moves the servos and motors into their start positions
     */
    public void teleopStart() {
        intakeSubsystem.clawGrab();
        intakeSubsystem.wristDown();
    }

    public void tryTransfer() {
        isTransfering = true;
        startTransfer = true;
        cancelTransfer = false;
    }

    public void tryWristHold() {
        if (!isTransfering) {
            driverCanControlClaw = false;
            intakeSubsystem.wristHold();
        }
    }

    public void tryWristDown() {
        if (!isTransfering) {
            driverCanControlClaw = true;
            intakeSubsystem.wristDown();
        }
    }

    public void tryPowerOnShooter() {
        shooterSubsystem.powerOn();
    }

    public void tryToggleShooter() {
        shooterSubsystem.toggleShooterPower();
    }

    public void tryPowerOffShooter() {
        shooterSubsystem.powerOff();
    }

    public void tryShootElement() {
        isShooting = true;
        shootingStart = true;
        shootingCancel = false;
        shootingAmount = -1;
    }

    public void tryShootElement(int amountOfArtifacts) {
        isShooting = true;
        shootingStart = true;
        shootingCancel = false;
        shootingAmount = amountOfArtifacts;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public void tryPrimeShoot() {
        shooterSubsystem.shooterArmDown();
        shootingCancel = true;
    }

    public void trySetCustomVelocity(double newVelo) {
        shooterSubsystem.setCustomVelocity(newVelo);
    }

    public void tryCancelTransfer() {
        cancelTransfer = true;
        startTransfer = false;
    }

    public void tryClawGrab() {
        if (driverCanControlClaw) {
            intakeSubsystem.clawGrab();
        }
    }

    public void tryClawRelease() {
        if (driverCanControlClaw) {
            intakeSubsystem.clawRelease();
        }
    }

    public void tryClawToggle() {
        if (driverCanControlClaw) {
            intakeSubsystem.clawToggle();
        }
    }

    public boolean isTransfering() {
        return isTransfering;
    }

    public void setState(OpModeStates newState) {
        if (currentState != newState) {
            stateStart = true;

            currentState = newState;
        }
    }

    public void tryOpenGate() {
        if (!isTransfering) {
            intakeSubsystem.openGate();
        }
    }

    public void tryToggleGate() {
        if (!isTransfering) {
            if (intakeSubsystem.isGateClosed()) {
                intakeSubsystem.openGate();
            } else {
                intakeSubsystem.closeGate();
            }
        }
    }

    public void tryCloseGate() {
        if (!isTransfering) {
            intakeSubsystem.closeGate();
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
                if (isTransfering) {
                    driverCanControlClaw = false;

                    if (cancelTransfer) {
                        transferScheduler.cancel();
                        intakeSubsystem.clawGrab();
                        intakeSubsystem.wristDown();

                        isTransfering = false;
                        cancelTransfer = false;
                        return;
                    }

                    // we need to actually schedule tasks after the driver presses rb,
                    // the reasoning as to why this code is run in update, and not in
                    // transfer is to make the code more consistent and readable.
                    if (startTransfer) {
                        transferScheduler.reset(true);

                        double startTime = 0;

                        if (!intakeSubsystem.isClawClosed()) {
                            transferScheduler
                                    .addMethod(0, () -> {
                                        intakeSubsystem.clawGrab();
                                        intakeSubsystem.wristDown();
                                        intakeSubsystem.openGate();
                                    });

                            startTime = 400;
                        }

                        transferScheduler
                                .addMethod(startTime, () -> {
                                    intakeSubsystem.clawGrab();
                                    intakeSubsystem.wristUp();
                                    intakeSubsystem.openGate();
                                })
                                .addMethod(startTime + 600, () -> {
                                    intakeSubsystem.clawRelease();
                                })
                                .addMethod(startTime + 900, () -> {
                                    intakeSubsystem.wristDown();
                                })
                                .addMethod(startTime + 1300, () -> {
                                    intakeSubsystem.closeGate();
                                    isTransfering = false;
                                    driverCanControlClaw = true;
                                });

                        startTransfer = false;
                    }

                    opMode.telemetry.addData("num sch: ", transferScheduler.scheduleMap.size());

                    transferScheduler.update();
                } else {
                    if (stateStart) {
                        intakeSubsystem.wristDown();
                    }
                }

                if (isShooting) {
                    if (!shootingCancel && (shootingAmount > 0 || shootingAmount == -1)) {
                        if (shooterSubsystem.ready()) {
                            shooterSubsystem.shooterArmUp();
                            shootingAmount--;
                        } else {
                            shooterSubsystem.shooterArmDown();
                        }
                    } else {
                        // don't put the shooter arm down,
                        // that needs to be done when the program needs to prime another ball
                        isShooting = false;
                        shootingStart = false;
                    }
                }
                break;

            case PARK:

                break;
        }

        stateStart = false;

        shooterSubsystem.update();
        intakeSubsystem.update();
    }
}