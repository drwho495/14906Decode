package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;

import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private OpModeOptions managerOptions = new OpModeOptions();
    private LinearOpMode opMode;
    private Follower follower = null;
    private AllianceSides side = Parameters.LAST_ALLIANCE_SIDE;

    private boolean isShooting = false;
    private final boolean hoodServoManual = true;

    private double teleopHeadingGoal = 0;
    private boolean aimAtGoal = true;

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

    public boolean isShooterOn() {
        return shooterSubsystem.isPoweredOn();
    }

    public void initialisePedroPathing() {
        follower = new Follower(opMode.hardwareMap);
    }

    public void trySetDrivePowers(double x, double y, double heading, boolean fieldCentric) {
        if (!follower.teleopDriveEnabled()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        follower.setTeleOpMovementVectors(x, y, heading, !fieldCentric);
    }

    public void tryEnableAutoHeading() {
        follower.setAutoHeadingState(true);
    }

    public Follower getFollower() {
        return follower;
    }

    public void tryDisableAutoHeading() {
        follower.setAutoHeadingState(false);
    }

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    public void tryResetIMU() {
        if (!follower.isBusy()) {
            try {
                follower.resetIMU();
            } catch (InterruptedException e) {
            }
        }
    }

    /*
     * set a heading goal while in teleop drive mode that will not change regardless of position
     */
    public void trySetConstantTeleopHeading(double heading) {
        aimAtGoal = false;
        teleopHeadingGoal = heading;
    }

    public void tryUseGoalAimHeading() {
        aimAtGoal = true;
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

    public Pose getPose() {
        return follower.getPose();
    }

    public void setManagerOptions(OpModeOptions newOptions) {
        managerOptions = newOptions;
    }

    private void internalRunPath(PathBuilder path, boolean correctAfterFinished) {
        follower.followPath(path.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Robot Heading: ", Math.toDegrees(follower.getPose().getHeading()));
            opMode.telemetry.update();

            update();
            follower.update();
        }
    }

    public void tryRunBlocking(PathBuilder path) {
        if (!follower.isBusy()) internalRunPath(path, true);
    }

    public void tryRunBlocking(PathBuilder path, boolean correctAfterFinished) {
        if (!follower.isBusy()) internalRunPath(path, correctAfterFinished);
    }

    public void safeSleep(double time) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < time && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            update();
            follower.update();
        }
    }

    public double getHeadingToGoal() {
        Pose robotPose = follower.getPose();
        Pose goalPos = Parameters.RED_SHOOTER_GOAL;

        if (side == AllianceSides.BLUE) {
            goalPos = Parameters.BLUE_SHOOTER_GOAL;
        }

        return (Math.atan2(goalPos.getY() - robotPose.getY(), goalPos.getX() - robotPose.getX()) + Math.toRadians(180));
    }

    public void setAllianceSide(AllianceSides newSide) {
        side = newSide;
    }

    public void update() {
        if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                if (isShooting) {
                    shooterSubsystem.openFinger();
                } else {
                    shooterSubsystem.closeFinger();
                }

                if (follower.getAutoHeadingState()) {
                    if (aimAtGoal) {
                        teleopHeadingGoal = this.getHeadingToGoal();
                    }

                    follower.setTeleopHeadingGoal(teleopHeadingGoal);
                }
                break;

            case PARK:

                break;
        }

        stateStart = false;

        if (follower != null) follower.update();
        shooterSubsystem.update();
        intakeSubsystem.update();
    }

    public Double[] getShooterVelocities() {
        return shooterSubsystem.getVelocities();
    }

    public AllianceSides getAllianceSide() {
        return side;
    }

    /*
     * set the power limit for the drivetrain motors in auto
     */
    public void trySetMaxPower(double newPower) {
        follower.setMaxPower(newPower);
    }

    public void tryBreakFollowing() {
        follower.breakFollowing();
    }
}