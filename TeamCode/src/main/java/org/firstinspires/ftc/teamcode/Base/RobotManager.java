package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Helpers.PointsCurve;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;
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
    private final PointsCurve rpmCurve = new PointsCurve();
    private final PointsCurve hoodCurve = new PointsCurve();
    List<LynxModule> hubs = null;

    private boolean isShooting = false;
    private boolean manualShooting = false;

    private double teleopHeadingGoal = 0;
    private double transferSpeed = .7;
    private boolean aimHoldPoint = false;
    private boolean aimAtGoal = true;
    private boolean stateStart = true;
    private boolean resetDistanceToGoal = false;
    private double distanceToGoal = 0;

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
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double rpmOffset = 0;
        double hoodOffset = 0;

        rpmCurve.addPoint(65, 4380 + rpmOffset);
        hoodCurve.addPoint(65, 55 + hoodOffset);

        rpmCurve.addPoint(83, 4650 + rpmOffset);
        hoodCurve.addPoint(83, 75 + hoodOffset);

        rpmCurve.addPoint(100, 4850 + rpmOffset);
        hoodCurve.addPoint(100, 67 + hoodOffset);

        rpmCurve.addPoint(140, 5500 + rpmOffset);
        hoodCurve.addPoint(140, 75 + hoodOffset);

        rpmCurve.buildCurve();
        hoodCurve.buildCurve();
    }

    private void clearCache() {
        if (hubs == null) return;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public double getTransferDisableTime() {
        return intakeSubsystem.getTransferLockTime();
    }

    public void enableManualShooting() {
        manualShooting = true;
    }

    public void disableManualShooting() {
        manualShooting = false;
    }

    public boolean isManualShooterMode() {
        return manualShooting;
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
        if (manualShooting) shooterSubsystem.setVelocity(velocity);
    }

    public void setHoodServoPos(double newPos) {
        if (manualShooting) {
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
            opMode.telemetry.addData("Shooter RPM Goal: ", shooterSubsystem.getTargetVelocity());
            opMode.telemetry.addData("Using Variable Heading: ", follower.followerHeadingIsVariable());
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

    public void setTransferSpeed(double transferSpeed) {
        this.transferSpeed = transferSpeed;
    }

    public void setAllianceSide(AllianceSides newSide) {
        side = newSide;
    }

    public void update() {
        if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;

        if (timer.time(TimeUnit.MILLISECONDS) % 7 == 0) {
            clearCache();
        }

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                double distanceToGoal = getDistanceToGoal();

                if (!manualShooting) {
                    shooterSubsystem.setVelocity(rpmCurve.getY(distanceToGoal));
                    shooterSubsystem.setHoodPos(hoodCurve.getY(distanceToGoal));
                }

                if (isShooting) {
                    intakeSubsystem.disableAutoDisableTransfer();

                    if (distanceToGoal >= 58) {
                        intakeSubsystem.setPowerLimits(1, distanceToGoal < 110 ? transferSpeed : .4);
                    } else {
                        intakeSubsystem.setPowerLimits(0, 0);
                    }
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
                break;

            case PARK:

                break;
        }

        if (follower.getAutoHeadingState()) {
            if (aimAtGoal) {
                teleopHeadingGoal = this.getHeadingToGoal();
            }

            follower.setTeleopHeadingGoal(teleopHeadingGoal);
        }

//        if (aimHoldPoint) {
//            follower.updateHoldPoint(new Pose(holdPointPosition.getX(), holdPointPosition.getY(), getHeadingToGoal()));
//        }

        stateStart = false;
        resetDistanceToGoal = true;

        Parameters.OPMODE_END_POSITION = follower.getPose();

        if (follower != null) follower.update();
        shooterSubsystem.update();
        intakeSubsystem.update();
    }

    /**
     * This method sets the new driver offset.
     * @param driverOffset New offset, in degrees.
     */
    public void setDriverOffset(double driverOffset) {
        follower.setDriverOffset(Math.toRadians(driverOffset));
    }

    /**
     * This method tells the robot to correct it's heading until it falls under a certain tolerance.
     *
     * @param tolerance The heading tolerance, in degrees.
     * @param timeout The amount of time to wait before giving up and letting the rest of the code run.
     * @param velocityConstraint The maximum velocity needed to end the loop.
     */
    public void waitForHeadingCorrection(double tolerance, double timeout, double velocityConstraint) {
        autoTimer.reset();

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            double error = Math.abs(follower.headingError);

            opMode.telemetry.addData("Heading Error: ", error);
            opMode.telemetry.addData("Heading: ", Math.toDegrees(getPose().getHeading()));
            opMode.telemetry.addData("Velocity: ", follower.getVelocityMagnitude());
            opMode.telemetry.update();

            if (autoTimer.time(TimeUnit.MILLISECONDS) > timeout || (error < Math.toRadians(tolerance) && Math.abs(follower.getVelocityMagnitude()) < velocityConstraint)) {
                break;
            }

            update();
            follower.update();
        }
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

    public double getDistanceToGoal() {
        if (resetDistanceToGoal) {
            Pose robotPose = follower.getPose();

            Pose goalPos = Parameters.RED_SHOOTER_GOAL;

            if (side == AllianceSides.BLUE) {
                goalPos = Parameters.BLUE_SHOOTER_GOAL;
            }

            resetDistanceToGoal = false;
            distanceToGoal =  MathFunctions.distance(robotPose, goalPos);
        }
        return distanceToGoal;
    }

    public boolean isTransferStalled() {
        return intakeSubsystem.isTransferStalled();
    }

    public int getHeldBallCount() {
        return intakeSubsystem.getHeldBallCount();
    }

    public void waitForShooter(double timeout) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();

        while (opMode.opModeIsActive() & waitTimer.time(TimeUnit.MILLISECONDS) < timeout) {
            if (shooterSubsystem.ready()) {
                break;
            }

            update();
        }
    }
}