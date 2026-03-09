package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import android.util.Pair;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Base.Helpers.HardwareUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.PointsCurve;
import org.firstinspires.ftc.teamcode.Base.Helpers.Polygon;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private LinearOpMode opMode;
    private Follower follower = null;
    private AllianceSides side = Parameters.LAST_ALLIANCE_SIDE;
    private double autoTimeout = -1;
    private Pose goalPos = Parameters.SHOOTER_GOAL_CLOSE;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    private Polygon closeZone = new Polygon();
    private Polygon farZone = new Polygon();
    private Polygon robotGeometricRepresentation;
    private final PointsCurve rpmCurve = new PointsCurve();
    private final PointsCurve hoodCurve = new PointsCurve();

    List<LynxModule> hubs = null;

    private boolean isShooting = false;
    private boolean manualShooterControl = false;
    private boolean waitForVelocityToShoot = true;
    private boolean onlyShootInZone = true;

    private double teleopHeadingGoal = 0;
    private boolean usingAutoHeading = false;
    private boolean autoTeleShooting = false;
    private boolean autoTeleShootingHeadingSwitch = false;
    private double autoTeleShootingMaxSpeed = 1;
    private boolean followerEndStalePathEnabled = true;
    private List<Pair<Double, Long>> velocityMagnitudes = new ArrayList<>();

    private double intakePower = 0;
    private double lastIntakePower = 0;
    private double transferSpeed = 1;
    private boolean activeHoldLastBallEnabled = true;
    private boolean activeHoldingLastBall = false;
    private ElapsedTime activeHoldingLastBallTimer = new ElapsedTime();

    private boolean aimHoldPoint = false;
    private boolean aimAtGoal = true;
    private boolean useHoodCompensation = false;
    private boolean useVelocityCompensation = true;
    private boolean stateStart = true;
    private boolean resetDistanceToGoal = false;
    private double distanceToGoal = 0;
    private boolean autoTransferStopEnabled = true;
    private boolean printDebugEnabled = false;
    private double pathGetHeadingToGoalTrackT = .75;
    private ShootingStyle shootingStyle = ShootingStyle.STANDARD;
    private boolean canShoot = false;
    private boolean canShootOveride = false;
    private double canShootAtVelocity = 0;

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

    public void enableAutoTransferStop() {
        autoTransferStopEnabled = true;
    }

    public void disableAutoTransferStop() {
        autoTransferStopEnabled = false;
    }

    public void enableDebugPrinting() {
        printDebugEnabled = true;
    }

    public void disableDebugPrinting() {
        printDebugEnabled = false;
    }

    public void enableVelocityCompensation() {
        useVelocityCompensation = true;
    }

    public void disableVelocityCompensation() {
        useVelocityCompensation = false;
    }

    public void enableWaitForVelocityToShoot() {
        waitForVelocityToShoot = true;
    }

    public void disableWaitForVelocityToShoot() {
        waitForVelocityToShoot = false;
    }

    public void disablePoweredHold() {
        activeHoldLastBallEnabled = false;
    }

    public void enablePoweredHold() {
        activeHoldLastBallEnabled = true;
    }

    public void printDebugInfo() {
        opMode.telemetry.addData("Robot Heading: ", Math.toDegrees(follower.getPose().getHeading()));
        opMode.telemetry.addData("Robot Heading Error: ", Math.toDegrees(follower.headingError));
        opMode.telemetry.addData("Alliance Side: ", side == AllianceSides.BLUE ? "Blue" : "Red");
        opMode.telemetry.addData("Shooter RPM Goal: ", shooterSubsystem.getTargetVelocity());
        if (follower.isBusy()) opMode.telemetry.addData("Using Variable Heading: ", follower.followerHeadingIsVariable());
        opMode.telemetry.update();
    }

    public void initialise() {
        shooterSubsystem.initialiseHardware();
        intakeSubsystem.initialiseHardware();

        follower = new Follower(opMode.hardwareMap);

        List<LynxModule> hubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        HardwareUtils.previousValues.clear();

        Pose closeEndPoint = new Pose(-48, -65);
        Pose closeGoalPoint = new Pose(10, -10);
        Pose farEndPoint = new Pose(-55, -100);

        closeZone.addPoint(closeEndPoint);
        closeZone.addPoint(closeGoalPoint);
        closeZone.addPoint(closeGoalPoint.getMirroredCopy());
        closeZone.addPoint(closeEndPoint);

        farZone.addPoint(farEndPoint);
        farZone.addPoint(new Pose(-20, -131));
        farZone.addPoint(new Pose(-85, -131));
        farZone.addPoint(farEndPoint);

        robotGeometricRepresentation = Polygon.makeRectangle(12, 18);

        setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        buildRPMCurves();
    }
    
    private void buildRPMCurves() {
        if (shootingStyle == ShootingStyle.STANDARD) {
            rpmCurve.addPoint(65, 4380);
            hoodCurve.addPoint(65, 55);

            rpmCurve.addPoint(81, 4590);
            hoodCurve.addPoint(81, 80);

            rpmCurve.addPoint(100, 4850);
            hoodCurve.addPoint(100, 67);

            rpmCurve.addPoint(140, 5100);
            hoodCurve.addPoint(140, 70);

            shooterSubsystem.setHoodCompensationMultiplier(4);
        } else if (shootingStyle == ShootingStyle.LARGE_ARC) {
            rpmCurve.addPoint(65, 3900);
            hoodCurve.addPoint(65, 20);

            rpmCurve.addPoint(76, 3950);
            hoodCurve.addPoint(76, 20);

            rpmCurve.addPoint(83, 4030);
            hoodCurve.addPoint(83, 20);

            rpmCurve.addPoint(100, 4280);
            hoodCurve.addPoint(100, 35);

            rpmCurve.addPoint(140, 5400);
            hoodCurve.addPoint(140, 85);

            shooterSubsystem.setHoodCompensationMultiplier(25);
        } else if (shootingStyle == ShootingStyle.UNJAM) {
            rpmCurve.addPoint(81, 4284);
            hoodCurve.addPoint(81, 120);

            rpmCurve.addPoint(81, 4284);
            hoodCurve.addPoint(81, 120);
        }

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

    public void enableManualShooterControl() {
        manualShooterControl = true;
    }

    public void disableManualShooterControl() {
        manualShooterControl = false;
    }

    public boolean isManualShooterMode() {
        return manualShooterControl;
    }

    public boolean isShooterOn() {
        return shooterSubsystem.isPoweredOn();
    }

    public void setDrivePowers(double x, double y, double heading, boolean fieldCentric) {
        if (!follower.teleopDriveEnabled()) {
            breakFollowing();
            follower.startTeleopDrive();
        }

        if (autoTeleShooting) {
            x = Range.clip(x, -autoTeleShootingMaxSpeed, autoTeleShootingMaxSpeed);
            y = Range.clip(y, -autoTeleShootingMaxSpeed, autoTeleShootingMaxSpeed);
        }

        follower.setTeleOpMovementVectors(x, y, heading, !fieldCentric);
    }

    public void enableAutoHeading() {
        usingAutoHeading = true;
    }

    public Follower getFollower() {
        return follower;
    }

    public void disableAutoHeading() {
        usingAutoHeading = false;
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
        setIntakePower(-1);
    }

    public void powerOffIntake() {
        setIntakePower(0);
    }

    public void setIntakePower(double newPower) {
        if (newPower != intakePower && lastIntakePower != intakePower) {
            lastIntakePower = intakePower;
        }

        intakePower = newPower;
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

    public void shootElements() {
        canShoot = false;
        canShootOveride = false;
        isShooting = true;
    }

    public void cancelShootElements() {
        isShooting = false;
    }

    public void enableAutomaticTeleopShooting() {
        autoTeleShooting = true;
        autoTeleShootingHeadingSwitch = false;
        autoTeleShootingMaxSpeed = 1;
    }

    public void disableAutomaticTeleopShooting() {
        autoTeleShooting = false;
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
        if (manualShooterControl) shooterSubsystem.setVelocity(velocity);
    }

    public void setHoodServoPos(double newPos) {
        if (manualShooterControl) {
            shooterSubsystem.setHoodPos(newPos);
        }
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setShootingStyle(ShootingStyle shootingStyle) {
        this.shootingStyle = shootingStyle;

        // rebuild the curves if the driver wants to change how they shoot mid-match
//        if (rpmCurve.isBuilt() || hoodCurve.isBuilt()) {
            buildRPMCurves();
//        }
    }

    public ShootingStyle getShootingStyle() {
        return this.shootingStyle;
    }

    private void internalRunPath(PathBuilder path, boolean correctAfterFinished) {
        follower.followPath(path.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            if (autoTimeout > 0 && autoTimer.time(TimeUnit.MILLISECONDS) >= autoTimeout) {
                breakFollowing(correctAfterFinished);
                break;
            }

            if (followerEndStalePathEnabled) {
                velocityMagnitudes.add(0, new Pair<>(follower.getVelocityMagnitude(), System.currentTimeMillis()));
                double startTime = -1;
                double summedVelocity = 0;
                int numberOfInstances = 0;

                for (Pair<Double, Long> pair : velocityMagnitudes) {
                    if (startTime == -1) {
                        startTime = pair.second;
                    }

                    numberOfInstances++;
                    summedVelocity += pair.first;

                    if (startTime - pair.second >= 500) {
                        if (numberOfInstances != 0 && Math.abs(summedVelocity / numberOfInstances) <= 3) {
                            breakFollowing(correctAfterFinished);
                        }

                        break;
                    }
                }
            }

            update();
            follower.update();
        }

        velocityMagnitudes.clear();
        autoTimeout = -1;
    }

    public void runBlocking(PathBuilder path) {
        if (!follower.isBusy()) internalRunPath(path, true);
    }

    public void runPassthrough(PathBuilder path) {
        follower.followPath(path.build());
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

        safeSleep(timer, time);
    }

    public void safeSleep(ElapsedTime timer, double time) {
        while (timer.time(TimeUnit.MILLISECONDS) < time && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            update();
            follower.update();
        }
    }

    public void addPathTimeout(double timeout) {
        autoTimeout = timeout;
        autoTimer.reset();
    }

    public void clearPathTimeout() {
        autoTimeout = -1;
        autoTimer.reset();
    }

    public Pose getVelocityCorrectedPose() {
        return follower.getPose().add(follower.getVelocity().returnMultiplied(.5).toPose());
    }

    public double getHeadingToGoal(Pose robotPose) {
        return getHeadingToPose(robotPose, goalPos);
    }

    public double getHeadingToPose(Pose robotPose, Pose targetPose) {
        return (Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()) + Math.PI);
    }

    public void stopAndAim() {
        breakFollowing();

        follower.startTeleopDrive();
        follower.setAutoHeadingState(true);

        enableAutoHeading();
        useGoalAimHeading();
    }

    public void enableOnlyShootInZone() {
        onlyShootInZone = true;
    }

    public void disableOnlyShootInZone() {
        onlyShootInZone = false;
    }

    public double getHeadingToGoal() {
        return getHeadingToGoal(useVelocityCompensation ? getVelocityCorrectedPose() : getPose());
    }

    public void recalibrateIMU() {
        follower.recalibrateIMU();
    }

    public double getHeadingToGoalWhileFollowingPath() {
        if (follower.isBusy()) {
            Path currentPath = follower.getCurrentPath();

            if (follower.getCurrentTValue() < pathGetHeadingToGoalTrackT) {
                Pose futurePose = new Pose();
                Point futurePoint = currentPath.getPoint(pathGetHeadingToGoalTrackT);

                futurePose.setX(futurePoint.getX());
                futurePose.setY(futurePoint.getY());

                return getHeadingToGoal(futurePose);
            }
        }
        return getHeadingToGoal();
    }

    public void setTransferSpeed(double transferSpeed) {
        this.transferSpeed = transferSpeed;
    }

    public void enableHoodCompensation() {
        useHoodCompensation = true;
    }

    public void disableHoodCompensation() {
        useHoodCompensation = false;
    }

    public double getTransferSpeed() {
        return transferSpeed;
    }

    public void setAllianceSide(AllianceSides newSide) {
        side = newSide;
        setShooterZone(Parameters.SHOOTER_GOAL_CLOSE);
    }

    public void setShooterZone(Pose newZone) {
        goalPos = getFixedPose(newZone);
    }

    public void update() {
        if ((!opMode.opModeIsActive() && !opMode.opModeInInit()) || opMode.isStopRequested()) return;

        if (timer.time(TimeUnit.MILLISECONDS) % 7 == 0) {
            clearCache();
        }

        if (usingAutoHeading) {
            follower.setAutoHeadingState(true);

            if (aimAtGoal) {
                teleopHeadingGoal = getHeadingToGoal();
            }

            follower.setTeleopHeadingGoal(teleopHeadingGoal);
        } else {
            follower.setAutoHeadingState(false);
        }

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                double distanceToGoal = getDistanceToGoal();

                robotGeometricRepresentation.setOffsets(getPose());

                if (!manualShooterControl) {
                    shooterSubsystem.setVelocity(rpmCurve.getY(distanceToGoal));
                    shooterSubsystem.setHoodPos(hoodCurve.getY(distanceToGoal));
                }

                boolean inZone = closeZone.contains(robotGeometricRepresentation) || farZone.contains(robotGeometricRepresentation);

                if (isShooting) {
                    boolean needsToWait = false;

                    if (waitForVelocityToShoot && !autoTeleShooting) {
                        if ((!shooterSubsystem.ready() || shooterSubsystem.getRateOfChange() < 1)
                                && !canShootOveride
                                && !(canShoot && Math.abs(canShootAtVelocity - shooterSubsystem.getTargetVelocity()) < 15))
                        {
                            needsToWait = true;
                        }
                    }

                    if (onlyShootInZone || autoTeleShooting) {
                        if (!inZone) {
                            needsToWait = true;
                            autoTeleShootingMaxSpeed = 1;
                        } else {
                            autoTeleShootingMaxSpeed = .7;
                        }
                    }

                    activeHoldingLastBall = false;

                    intakeSubsystem.disableAutoDisableTransfer();
                    intakeSubsystem.setIntakePower(intakePower);

                    if (useHoodCompensation) {
                        shooterSubsystem.enableHoodCompensation();
                    } else {
                        shooterSubsystem.disableHoodCompensation();
                    }

                    if (!needsToWait) {
                        if (!canShoot) {
                            canShoot = true;
                            canShootAtVelocity = shooterSubsystem.getTargetVelocity();
                        }

                        if (intakeSubsystem.getIntakePower() >= .5 || autoTeleShooting) {
                            canShootOveride = true;
                        }

                        if (autoTeleShooting) {
                            // if we can shoot then we NEED to shoot
                            intakeSubsystem.setIntakePower(1);
                        }

                        if (distanceToGoal >= Parameters.MIN_SHOOT_DISTANCE) {
                            if (distanceToGoal < Parameters.FAR_ZONE_DISTANCE) {
                                shooterSubsystem.usePrimaryPF();
//                                setShooterZone(Parameters.SHOOTER_GOAL_FAR);
                                intakeSubsystem.setPowerLimits(1, transferSpeed);
                            } else {
                                shooterSubsystem.useSecondaryPF();
//                                setShooterZone(Parameters.SHOOTER_GOAL_CLOSE);
                                intakeSubsystem.setPowerLimits(1, Parameters.SLOW_TRANSFER);
                            }
                        } else {
                            intakeSubsystem.setPowerLimits(0, 0);
                        }

                        shooterSubsystem.openFinger();
                    } else {
                        canShoot = false;
                        shooterSubsystem.closeFinger();

                        intakeSubsystem.setPowerLimits(.5, 0);
                    }
                } else {
                    shooterSubsystem.disableHoodCompensation();

                    if (intakePower == 0 && activeHoldLastBallEnabled && lastIntakePower > 0) {
                        if (!activeHoldingLastBall) {
                            activeHoldingLastBallTimer.reset();
                            activeHoldingLastBall = true;
                        }

                        if (activeHoldingLastBallTimer.time(TimeUnit.MILLISECONDS) < 750) {
                            intakeSubsystem.setIntakePowers(.25, 0);
                        } else {
                            intakeSubsystem.setIntakePower(0);
                        }
                    } else {
                        activeHoldingLastBall = false;

                        intakeSubsystem.setIntakePower(intakePower);
                    }

                    if (intakePower > 0) {
                        if (autoTransferStopEnabled) {
                            intakeSubsystem.enableAutoDisableTransfer();
                        } else {
                            intakeSubsystem.disableAutoDisableTransfer();
                        }
                    } else if (intakePower < 0) {
                        intakeSubsystem.disableAutoDisableTransfer();
                    }

                    intakeSubsystem.setPowerLimits(1, 1);
                    shooterSubsystem.closeFinger();
                }
                break;

            case PARK:

                break;
        }

        if (printDebugEnabled)
            printDebugInfo();

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

        clearPathTimeout();
    }

    public void breakFollowing(boolean holdPoint) {
        Path lastPath = null;
        Point lastPoint = null;
        Pose lastPose = null;

        if (holdPoint && follower.isBusy()) {
            lastPath = follower.getCurrentPath();
            lastPoint = lastPath.getLastControlPoint();
            lastPose = new Pose(lastPoint.getX(), lastPoint.getY(), lastPath.getLastHeadingInterpolation().getEndHeading());
        }

        breakFollowing();

        if (holdPoint) {
            follower.holdPoint(lastPose);
        }
    }

    public double getHoodAngle() {
        return shooterSubsystem.getHoodAngle();
    }

    public double getDistanceToGoal() {
        if (resetDistanceToGoal) {
            Pose robotPose = useVelocityCompensation ? getVelocityCorrectedPose() : getPose();

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

    public boolean shooterReady() {
        return shooterSubsystem.ready();
    }

    public void turnTo(double headingGoal, double error) {
        Pose targetPose = getPose();

        targetPose.setHeading(headingGoal);

        follower.holdPoint(targetPose);
        follower.update();

        while (opMode.opModeIsActive() && Math.abs(follower.headingError) > error) {
            update();
        }
    }
}