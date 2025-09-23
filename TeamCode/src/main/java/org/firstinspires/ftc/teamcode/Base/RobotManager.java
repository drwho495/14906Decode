package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.Helpers.Scheduler;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private OpModeOptions managerOptions = new OpModeOptions();
    private LinearOpMode opMode;
    private boolean isTransfering = false;
    private boolean startTransfer = true;
    private boolean cancelTransfer = false;
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
                                    });

                            startTime = 400;
                        }

                        transferScheduler
                                .addMethod(startTime, () -> {
                                    intakeSubsystem.clawGrab();
                                    intakeSubsystem.wristUp();
                                })
                                .addMethod(startTime + 600, () -> {
                                    intakeSubsystem.clawRelease();
                                })
                                .addMethod(startTime + 1200, () -> {
                                    intakeSubsystem.wristDown();
                                    isTransfering = false;
                                    driverCanControlClaw = true;
                                });

                        startTransfer = false;
                    }

                    opMode.telemetry.addData("num sch: ", transferScheduler.scheduleMap.size());

                    if (transferScheduler.finished()) {
//                        isTransfering = false;
                    }

                    transferScheduler.update();
                } else {
                    driverCanControlClaw = true;
                    if (stateStart) {
                        intakeSubsystem.wristDown();
                    }
                }
                break;

            case PARK:

                break;
        }

        shooterSubsystem.update();
        intakeSubsystem.update();
    }
}