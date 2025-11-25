package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private LinearOpMode opMode;
    private Follower follower = null;
    private AllianceSides side = Parameters.LAST_ALLIANCE_SIDE;
    private double autoTimeout = -1;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    List<LynxModule> hubs;

    private boolean isShooting = false;
    private final boolean hoodServoManual = true;

    private double teleopHeadingGoal = 0;
    private boolean aimAtGoal = true;
    private boolean stateStart = true;

    // do NOT add a constructor to any of the subsystems!
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotManager(LinearOpMode newOpMode) {
        opMode = newOpMode;
        shooterSubsystem.setLinearTeleop(this.opMode);
        intakeSubsystem.setLinearTeleop(this.opMode);
    }

    public OpModeStates getState() {
        return currentState;
    }

    public void initialise() {
        shooterSubsystem.initialiseHardware();
        intakeSubsystem.initialiseHardware();

        follower = new Follower(opMode.hardwareMap);

        List<LynxModule> hubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isShooterOn() {
        return shooterSubsystem.isPoweredOn();
    }

    public void setDrivePowers(double x, double y, double heading, boolean fieldCentric) {
        if (!follower.teleopDriveEnabled()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        follower.setTeleOpMovementVectors(x, y, heading, !fieldCentric);
    }

    public void enableAutoHeading() {
        follower.setAutoHeadingState(true);
    }

    public Follower getFollower() {
        return follower;
    }

    public void disableAutoHeading() {
        follower.setAutoHeadingState(false);
    }

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    public void resetIMU() {
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
    public void setConstantTeleopHeading(double heading) {
        aimAtGoal = false;
        teleopHeadingGoal = heading;
    }

    public void useGoalAimHeading() {
        aimAtGoal = true;
    }

    public void powerOnShooter() {
        shooterSubsystem.powerOn();
    }

    public void reverseIntake() {
//        if (!isShooting) {
        intakeSubsystem.setIntakePower(-1);
//        }
    }

    public void powerOffIntake() {
//        if (!isShooting) {
        intakeSubsystem.powerIntakeOff();
//        }
    }

    public void setIntakePower(double newPower) {
//        if (!isShooting) {
        intakeSubsystem.setIntakePower(newPower);
//        }
    }

    public boolean isIntakeOn() {
        return intakeSubsystem.isIntakeOn();
    }

    public void toggleShooter() {
        shooterSubsystem.toggleShooterPower();
    }

    public void powerOffShooter() {
        shooterSubsystem.powerOff();
    }

    public void startShootElement() {
        isShooting = true;
    }

    public void stopShootElement() {
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

    public void setShooterVelocity(double velocity) {
        shooterSubsystem.setVelocity(velocity);
    }

    public void setHoodServoPos(double newPos) {
        if (hoodServoManual) {
            shooterSubsystem.setHoodPos(newPos);
        }
    }

    public Pose getPose() {
        return follower.getPose();
    }

    private void internalRunPath(PathBuilder path, boolean correctAfterFinished) {
        follower.followPath(path.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Robot Heading: ", Math.toDegrees(follower.getPose().getHeading()));
            opMode.telemetry.addData("Alliance Side: ", side == AllianceSides.BLUE ? "Blue" : "Red");
            opMode.telemetry.update();

            if (autoTimeout > 0 && autoTimer.time(TimeUnit.MILLISECONDS) > autoTimeout) {
                follower.breakFollowing();
                break;
            }

            update();
            follower.update();
        }

        autoTimeout = -1;
    }

    public void runBlocking(PathBuilder path) {
        if (!follower.isBusy()) internalRunPath(path, true);
    }

    /*
     * This will get a copy of the inputted pose that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Pose getFixedPose(Pose pose) {
        if (side == AllianceSides.RED) {
            return pose; // no mirroring is needed
        } else {
            return pose.getMirroredCopy();
        }
    }

    /*
     * This will get a copy of the inputted x, y and heading values that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Pose getFixedPose(double x, double y, double heading) {
        Pose builtPose = new Pose(x, y, heading);

        if (side == AllianceSides.RED) {
            return builtPose; // no mirroring is needed
        } else {
            return builtPose.getMirroredCopy();
        }
    }

    /*
     * This will get a copy of the inputted heading (in degrees!) value that will/won't be mirrored
     * depending on the robot's alliance
     */
    public double getFixedHeading(double heading) {
        if (side == AllianceSides.RED) {
            return Math.toRadians(heading); // no mirroring is needed
        } else {
            return new Pose(0, 0, Math.toRadians(heading)).getMirroredCopy().getHeading();
        }
    }

    /*
     * This will get a copy of the inputted point that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Point getFixedPoint(Point point) {
        if (side == AllianceSides.RED) {
            return point; // no mirroring is needed
        } else {
            return new Point(new Pose(point.getX(), point.getY()).getMirroredCopy());
        }
    }

    /*
     * This will get a copy of the inputted pose that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Point getFixedPoint(Pose pose) {
        if (side == AllianceSides.RED) {
            return new Point(pose); // no mirroring is needed
        } else {
            return new Point(pose.getMirroredCopy());
        }
    }

    /*
     * This will get a copy of the inputted x and y values that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Point getFixedPoint(double x, double y) {
        if (side == AllianceSides.RED) {
            return new Point(x, y, Point.CARTESIAN); // no mirroring is needed
        } else {
            return new Point(new Pose(x, y).getMirroredCopy());
        }
    }

    public void runBlocking(PathBuilder path, boolean correctAfterFinished) {
        if (!follower.isBusy()) internalRunPath(path, correctAfterFinished);
    }

    public void safeSleep(double time) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < time && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            update();
            follower.update();
        }
    }

    public void addPathTimeout(double timeout) {
        autoTimeout = timeout;
        autoTimer.reset();
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

        if (timer.time(TimeUnit.MILLISECONDS) % 5 == 0) {
            clearCache();
        }

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                if (isShooting) {
                    intakeSubsystem.disableAutoDisableTransfer();

                    intakeSubsystem.setPowerLimits(1, .7);
                    shooterSubsystem.openFinger();
                } else {
                    if (intakeSubsystem.getIntakePower() > 0) {
                        intakeSubsystem.enableAutoDisableTransfer();
                    } else if (intakeSubsystem.getIntakePower() < 0) {
                        intakeSubsystem.disableAutoDisableTransfer();
                    }

                    intakeSubsystem.setPowerLimits(1, 1);
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

    public Double[] getCurrentShooterVelocities() {
        return shooterSubsystem.getVelocities();
    }

    public double getShooterTargetVelocity() {
        return shooterSubsystem.getTargetVelocity();
    }

    public AllianceSides getAllianceSide() {
        return side;
    }

    /*
     * set the power limit for the drivetrain motors in auto
     */
    public void setMaxFollowerPower(double newPower) {
        follower.setMaxPower(newPower);
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }

    public double getHoodAngle() {
        return shooterSubsystem.getHoodAngle();
    }
}