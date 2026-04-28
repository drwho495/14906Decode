package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.pedropathing.VectorCalculator;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.HardwareUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.PedroUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.Polygon;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterPFState;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;

import com.pedropathing.geometry.Pose;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private LinearOpMode opMode;
    private Follower follower = null;
    private VectorCalculator vectorCalculator = null;
    private Drivetrain drivetrain;
    private AllianceSides side = Parameters.LAST_ALLIANCE_SIDE;
    private double autoTimeout = -1;
    private double goalOffset = 0;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

    private Polygon closeZone = new Polygon();
    private Polygon farZone = new Polygon();
    private Polygon robotGeometricRepresentation;

    List<LynxModule> hubs = null;

    private boolean isShooting = false;
    private ShooterControlPolicy shooterControlPolicy = ShooterControlPolicy.ROAMING;
    private boolean waitForVelocityToShoot = true;
    private boolean onlyShootInZone = false;

    private double teleopHeadingGoal = 0;
    private boolean usingTeleopHeadingLock = false;
    private boolean autoTeleShooting = false;
    private boolean followerEndStalePathEnabled = true;
    private double teleopHeadingOffset = 0;

    private double intakePower = 0;
    private double lastIntakePower = 0;
    private double transferSpeed = 1;
    private boolean activeHoldLastBallEnabled = true;
    private boolean activeHoldingLastBall = false;
    private final ElapsedTime activeHoldingLastBallTimer = new ElapsedTime();

    private boolean aimHoldPoint = false;
    private boolean aimAtGoal = true;
    private boolean useHoodCompensation = false;
    private boolean useVelocityCompensation = true;
    private boolean stateStart = true;
    private boolean resetDistanceToGoal = false;
    private boolean autoTransferStopEnabled = true;
    private boolean printDebugEnabled = false;
    private ShootingStyle shootingStyle = ShootingStyle.LARGE_ARC;
    private boolean canShoot = false;
    private boolean canShootOveride = false;
    private boolean isTransferStopped = false;
    private final ElapsedTime transferStopTimer = new ElapsedTime();
    private boolean transferStopTimerPaused = true;
    private double canShootAtVelocity = 0;
    private double pathGetHeadingToGoalTrackT = .75;
    private double distanceToGoal = 0;

    // do NOT add a constructor to any of the subsystems!
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotManager(LinearOpMode newOpMode) {
        opMode = newOpMode;
        shooterSubsystem.setLinearTeleop(this.opMode);
        intakeSubsystem.setLinearTeleop(this.opMode);
    }

    public void setShooterControlPolicy(ShooterControlPolicy newControlPolicy) {
        shooterControlPolicy = newControlPolicy;
    }

    public ShooterControlPolicy getShooterControlPolicy() {
        return shooterControlPolicy;
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
        opMode.telemetry.addData("Alliance Side: ", side == AllianceSides.BLUE ? "Blue" : "Red");
        opMode.telemetry.addData("Shooter RPM Goal: ", shooterSubsystem.getTargetVelocity());
        opMode.telemetry.addData("Hood Servo Angle: ", shooterSubsystem.getHoodAngle());
        opMode.telemetry.update();
    }

    public void initialise() {
        shooterSubsystem.initialiseHardware();
        intakeSubsystem.initialiseHardware();

        follower = PedroConstants.getFollower(opMode.hardwareMap);
        vectorCalculator = follower.getVectorCalculator();
        drivetrain = follower.getDrivetrain();

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
        closeZone.addPoint(PedroUtils.getMirroredPose(closeGoalPoint));
        closeZone.addPoint(closeEndPoint);

        farZone.addPoint(farEndPoint);
        farZone.addPoint(new Pose(-20, -131));
        farZone.addPoint(new Pose(-85, -131));
        farZone.addPoint(farEndPoint);

        robotGeometricRepresentation = Polygon.makeRectangle(12, 18);

        setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        buildShooterCurves();
    }

    public void addGoalOffset(double numberToAdd) {
        goalOffset += numberToAdd;
    }

    public void resetGoalOffset() {
        setGoalOffset(getAllianceSide() == AllianceSides.RED ? Parameters.DEFAULT_AIM_OFFSET_RED : Parameters.DEFAULT_AIM_OFFSET_BLUE);
    }

    public void setGoalOffset(double newOffset) {
        goalOffset = newOffset;
    }

    public double getGoalOffset() {
        return goalOffset;
    }

    private Pose getOffsetedGoalPose() {
        Pose offsetedPose = getFixedPose(getGoalPosition().copy());

        double offsetX = offsetedPose.getX() + (goalOffset + 6);
        double offsetY = offsetedPose.getY() - goalOffset;

        if (offsetY < -6) {
            offsetedPose = offsetedPose.withY(offsetY + 6);
        } else {
            offsetedPose = offsetedPose.withX(offsetX);
        }

        return getFixedPose(offsetedPose);
    }

    public void buildShooterCurves() {
        Parameters.CLOSE_ZONE_CURVE.clear();
        Parameters.FAR_ZONE_CURVE.clear();

        if (shootingStyle == ShootingStyle.LARGE_ARC) {
            Parameters.CLOSE_ZONE_CURVE.addPoint(55, 20, 3400);
            Parameters.CLOSE_ZONE_CURVE.addPoint(65, 20, 3520);
            Parameters.CLOSE_ZONE_CURVE.addPoint(76, 20, 3580);
            Parameters.CLOSE_ZONE_CURVE.addPoint(83, 20, 3780);
            Parameters.CLOSE_ZONE_CURVE.addPoint(100, 30, 4300);

            Parameters.FAR_ZONE_CURVE.addPoint(110, 90, 5400);
            Parameters.FAR_ZONE_CURVE.addPoint(120, 90, 5450);

            shooterSubsystem.setHoodCompensationMultiplier(6);
        }


        Parameters.CLOSE_ZONE_CURVE.build();
        Parameters.FAR_ZONE_CURVE.build();
    }

    private void clearCache() {
        if (hubs == null) return;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isShooterOn() {
        return shooterSubsystem.isPoweredOn();
    }

    public void setDrivePowers(double x, double y, double heading, boolean fieldCentric) {
        if (follower.isBusy()) {
            breakFollowing(false);
        }

        Pose robotPose = getPose();
        Vector headingVector = new Vector();
        Vector driveVector = new Vector();

        if (usingTeleopHeadingLock) {
            double robotHeading = MathFunctions.normalizeAngle(robotPose.getHeading());
            double direction = MathFunctions.getTurnDirection(robotHeading, teleopHeadingGoal);

            teleopHeadingGoal = MathFunctions.normalizeAngle(teleopHeadingGoal);

            headingVector = vectorCalculator.getHeadingVector(
                    MathFunctions.getSmallestAngleDifference(robotHeading, teleopHeadingGoal) * direction,
                    robotPose,
                    teleopHeadingGoal
            );
        } else {
            headingVector.setComponents(heading, robotPose.getHeading());
        }

        driveVector.setOrthogonalComponents(x, y);
        driveVector.setMagnitude(Range.clip(driveVector.getMagnitude(), 0, 1));

        if (fieldCentric) {
            driveVector.rotateVector(teleopHeadingOffset);
        } else {
            driveVector.rotateVector(getPose().getHeading());
        }

        drivetrain.runDrive(
                new Vector(),
                headingVector,
                driveVector,
                robotPose.getHeading(),
                follower.getVelocity()
        );
    }

    public void enableAutoHeading() {
        usingTeleopHeadingLock = true;
    }

    public Follower getFollower() {
        return follower;
    }

    public void disableAutoHeading() {
        usingTeleopHeadingLock = false;
    }

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    public void resetIMU() {
        if (!follower.isBusy()) {
            follower.setHeading(0);
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

    public void startScoringCycle() {
        canShoot = false;
        canShootOveride = false;
        isShooting = true;
    }

    public void stopScoringCycle() {
        isShooting = false;
    }

    public void enableAutoShooterControl() {
        shooterControlPolicy = ShooterControlPolicy.ROAMING;
    }

    public void enableManualShooterControl() {
        shooterControlPolicy = ShooterControlPolicy.MANUAL;
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
        if (shooterControlPolicy == ShooterControlPolicy.MANUAL)
            shooterSubsystem.setVelocity(velocity);
    }

    public void setHoodServoPos(double newPos) {
        if (shooterControlPolicy == ShooterControlPolicy.MANUAL)
            shooterSubsystem.setHoodPos(newPos);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setShootingStyle(ShootingStyle shootingStyle) {
        this.shootingStyle = shootingStyle;

        // rebuild the curves if the driver wants to change how they shoot mid-match
        buildShooterCurves();
    }

    public ShootingStyle getShootingStyle() {
        return this.shootingStyle;
    }

    private void internalRunPath(PathBuilder path, boolean correctAfterFinished) {
        breakFollowing();

        follower.followPath(path.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            if (autoTimeout > 0 && autoTimer.time(TimeUnit.MILLISECONDS) >= autoTimeout) {
                breakFollowing(correctAfterFinished);
                break;
            }

            if (followerEndStalePathEnabled) {
//                velocityMagnitudes.add(0, new Pair<>(follower.getVelocity().getMagnitude(), System.currentTimeMillis()));
//                double startTime = -1;
//                double summedVelocity = 0;
//                int numberOfInstances = 0;
//
//                for (Pair<Double, Long> pair : velocityMagnitudes) {
//                    if (startTime == -1) {
//                        startTime = pair.second;
//                    }
//
//                    numberOfInstances++;
//                    summedVelocity += pair.first;
//
//                    if ((startTime - pair.second) >= 800) {
//                        if (numberOfInstances != 0 && Math.abs(summedVelocity / numberOfInstances) <= 1.5) {
//                            breakFollowing(correctAfterFinished);
//                        }
//
//                        break;
//                    }
//                }

                if (follower.isRobotStuck()) {
                    breakFollowing(correctAfterFinished);
                }
            }

            update();
            follower.update();
        }

        autoTimeout = -1;
    }

    public void runBlocking(PathBuilder path) {
        if (!follower.isBusy()) internalRunPath(path, true);
    }

    public void runPassthrough(PathBuilder path) {
        breakFollowing();

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
            return PedroUtils.getMirroredPose(pose);
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
            return PedroUtils.getMirroredPose(builtPose);
        }
    }

    /*
     * This will get a copy of the inputted x and y values that will/won't be mirrored
     * depending on the robot's alliance
     */
    public Pose getFixedPose(double x, double y) {
        return getFixedPose(x, y, 0);
    }

    /*
     * This will get a copy of the inputted heading (in degrees!) value that will/won't be mirrored
     * depending on the robot's alliance
     */
    public double getFixedHeading(double heading) {
        if (side == AllianceSides.RED) {
            return Math.toRadians(heading); // no mirroring is needed
        } else {
            return PedroUtils.getMirroredPose(new Pose(0, 0, Math.toRadians(heading))).getHeading();
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
        final double multiplier = .6;

        Vector velocity = follower.getVelocity();
        Pose robotPose = getPose();

        return robotPose
                .plus(
                        new Pose(
                                velocity.getXComponent() * multiplier,
                                velocity.getYComponent() * multiplier
                        )
                );
    }

    public double getHeadingToGoal(Pose robotPose) {
        return getHeadingToPose(robotPose, getOffsetedGoalPose());
    }

    public double getHeadingToPose(Pose robotPose, Pose targetPose) {
        return (Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()) + Math.PI);
    }

    public void stopAndAim() {
        breakFollowing(false);

        follower.holdPoint(follower.getPose().setHeading(getHeadingToGoal(follower.getPose())));
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
        if (PedroConstants.pinpointLocalizer != null) {
            PedroConstants.pinpointLocalizer.resetIMU();
            PedroConstants.pinpointLocalizer.recalibrate();
            PedroConstants.pinpointLocalizer.resetIMU();
        }
    }

//    public double getHeadingToGoalWhileFollowingPath() {
//        if (follower.isBusy()) {
//            Path currentPath = follower.getCurrentPath();
//
//            if (follower.getCurrentTValue() < pathGetHeadingToGoalTrackT) {
//                Pose futurePose = new Pose();
//                Point futurePoint = currentPath.getPoint(pathGetHeadingToGoalTrackT);
//
//                futurePose.setX(futurePoint.getX());
//                futurePose.setY(futurePoint.getY());
//
//                return getHeadingToGoal(futurePose);
//            }
//        }
//        return getHeadingToGoal();
//    }

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

    public boolean isTransferStopped() {
        return isTransferStopped;
    }

    public double transferOverCurrentTime() {
        if (transferStopTimerPaused) {
            return 0;
        } else {
            return transferStopTimer.time(TimeUnit.MILLISECONDS);
        }
    }

    public void setAllianceSide(AllianceSides newSide) {
        side = newSide;
        resetGoalOffset();

        Parameters.LAST_ALLIANCE_SIDE = newSide;
    }

    public void updateShooterParameters(double distanceToGoalInput) {
        if (distanceToGoalInput <= Parameters.FAR_ZONE_DISTANCE) {
            shooterSubsystem.setVelocity(Parameters.CLOSE_ZONE_CURVE.getRPMCurveOutput(distanceToGoalInput));
            shooterSubsystem.setHoodPos(Parameters.CLOSE_ZONE_CURVE.getHoodCurveOutput(distanceToGoalInput));
        } else {
            shooterSubsystem.setVelocity(Parameters.SHOOTER_FAR_ZONE_VELOCITY);
            shooterSubsystem.setHoodPos(Parameters.SHOOTER_FAR_ZONE_HOOD_ANGLE);
        }
    }

    public void updateShooterParameters(Pose robotPose) {
        updateShooterParameters(getDistanceToGoal(robotPose));
    }

    public void update() {
        if ((!opMode.opModeIsActive() && !opMode.opModeInInit()) || opMode.isStopRequested())
            return;

        if (timer.time(TimeUnit.MILLISECONDS) % 5 == 0) {
            clearCache();
        }

        if (usingTeleopHeadingLock) {
            if (aimAtGoal) {
                teleopHeadingGoal = getHeadingToGoal();
            }
        }

        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:
                robotGeometricRepresentation.setOffsets(getPose());

                if (shooterControlPolicy == ShooterControlPolicy.ROAMING) {
                    updateShooterParameters(getDistanceToGoal());
                }

                boolean inZone = closeZone.contains(robotGeometricRepresentation) || farZone.contains(robotGeometricRepresentation);

                if (isShooting) {
                    boolean needsToWait = false;

                    isTransferStopped = false;
                    transferStopTimerPaused = true;

                    if (waitForVelocityToShoot) {
                        if (!shooterSubsystem.ready() && !canShootOveride) {
                            needsToWait = true;
                        }
                    }

//                    if (onlyShootInZone || autoTeleShooting) {
//                        if (!inZone) {
//                            needsToWait = true;
//                        }
//                    }

                    activeHoldingLastBall = false;

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

                        if (intakeSubsystem.getIntakePower() >= .1) {
                            canShootOveride = true;
                            double distanceToGoal = getDistanceToGoal();

                            if (distanceToGoal >= Parameters.MIN_SHOOT_DISTANCE) {
                                if (distanceToGoal < Parameters.FAR_ZONE_DISTANCE) {
                                    shooterSubsystem.setPFState(ShooterPFState.TRANSFER_LOOP);
                                    intakeSubsystem.setPowerLimits(1, transferSpeed);
                                } else {
                                    double localTransferSpeed = Parameters.FAR_ZONE_TRANSFER_SPEED;

                                    if (transferSpeed < localTransferSpeed)
                                        localTransferSpeed = transferSpeed;

                                    shooterSubsystem.setPFState(ShooterPFState.FAST_TRANSFER_LOOP);
                                    intakeSubsystem.setPowerLimits(1, localTransferSpeed);
                                }
                            } else {
                                intakeSubsystem.setPowerLimits(0, 0);
                            }
                        } else {
                            canShootOveride = false;
                        }

                        shooterSubsystem.openFinger();
                    } else {
                        canShoot = false;
                        shooterSubsystem.closeFinger();

                        intakeSubsystem.setPowerLimits(.5, 0);
                    }
                } else {
                    shooterSubsystem.setPFState(ShooterPFState.WANDERING_LOOP);
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

                        double intakeMotor2Power = intakePower;

                        if (intakePower > 0) {
                            if (autoTransferStopEnabled) {
                                if (isTransferStopped) {
                                    intakeMotor2Power = 0;
                                } else {
                                    if (intakeSubsystem.isTransferOverCurrent(CurrentUnit.AMPS, 3)) {
                                        if (transferStopTimerPaused) {
                                            transferStopTimer.reset();
                                            transferStopTimerPaused = false;
                                        }

                                        if (transferStopTimer.time(TimeUnit.MILLISECONDS) >= 650) {
                                            isTransferStopped = true;
                                            transferStopTimerPaused = true;

                                            intakeMotor2Power = 0;
                                        }
                                    } else {
                                        transferStopTimerPaused = true;
                                    }
                                }
                            } else {
                                isTransferStopped = false;
                                transferStopTimerPaused = true;
                            }
                        } else {
                            isTransferStopped = false;
                            transferStopTimerPaused = true;
                            intakeSubsystem.setIntakePower(intakePower);
                        }

                        intakeSubsystem.setIntakePowers(intakePower, intakeMotor2Power);
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

        follower.update();
        shooterSubsystem.update();
        intakeSubsystem.update();
    }

    /**
     * This method sets the new driver offset.
     *
     * @param driverHeadingOffset New offset, in degrees.
     */
    public void setDriverOffset(double driverHeadingOffset) {
        teleopHeadingOffset = Math.toRadians(driverHeadingOffset);
    }

    /**
     * This method tells the robot to correct it's heading until it falls under a certain tolerance.
     *
     * @param tolerance          The heading tolerance, in degrees.
     * @param timeout            The amount of time to wait before giving up and letting the rest of the code run.
     * @param velocityConstraint The maximum velocity needed to end the loop.
     */
    public void waitForHeadingCorrection(double tolerance, double timeout, double velocityConstraint) {
        autoTimer.reset();

        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            double error = Math.abs(follower.getHeadingError());

            opMode.telemetry.addData("Heading Error: ", error);
            opMode.telemetry.addData("Heading: ", Math.toDegrees(getPose().getHeading()));
            opMode.telemetry.addData("Velocity: ", follower.getVelocity().getMagnitude());
            opMode.telemetry.update();

            if (autoTimer.time(TimeUnit.MILLISECONDS) > timeout || (error < Math.toRadians(tolerance) && Math.abs(follower.getVelocity().getMagnitude()) < velocityConstraint)) {
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
     * set the power limitMagnitude for the drivetrain motors in auto
     */
    public void setMaxFollowerPower(double newPower) {
        follower.setMaxPower(newPower);
    }

    public void breakFollowing() {
        breakFollowing(false);
    }

    public void breakFollowing(boolean holdPoint) {
        Pose lastPose = null;

        if (holdPoint && follower.isBusy()) {
            lastPose = follower.getCurrentPath().endPose();
        }

        follower.breakFollowing();
        clearPathTimeout();

        if (holdPoint) {
            follower.holdPoint(lastPose);
        }
    }

    public double getHoodAngle() {
        return shooterSubsystem.getHoodAngle();
    }

    public Pose getGoalPosition(boolean checkDistance) {
        Pose robotPose = new Pose();

        if (checkDistance)
            robotPose = getPose();

        if (getAllianceSide() == AllianceSides.RED) {
            if (!checkDistance || robotPose.distanceFrom(Parameters.SHOOTER_GOAL_CLOSE_RED) < Parameters.FAR_ZONE_DISTANCE) {
                return Parameters.SHOOTER_GOAL_CLOSE_RED;
            } else {
                return Parameters.SHOOTER_GOAL_FAR_RED;
            }
        } else {
            if (!checkDistance || robotPose.distanceFrom(Parameters.SHOOTER_GOAL_CLOSE_BLUE) < Parameters.FAR_ZONE_DISTANCE) {
                return Parameters.SHOOTER_GOAL_CLOSE_BLUE;
            } else {
                return Parameters.SHOOTER_GOAL_FAR_BLUE;
            }
        }
    }

    public Pose getGoalPosition() {
        return getGoalPosition(true);
    }

    public double getDistanceToGoal() {
        if (resetDistanceToGoal) {
            Pose robotPose = useVelocityCompensation ? getVelocityCorrectedPose() : getPose();

            resetDistanceToGoal = false;
            distanceToGoal = robotPose.distanceFrom(getGoalPosition());
        }
        return distanceToGoal;
    }

    public double getDistanceToGoal(Pose robotPosition) {
        return robotPosition.distanceFrom(getGoalPosition(false));
    }

    public void waitForShooter() {
        waitForShooter(-1);
    }

    public void waitForShooter(double timeout) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();

        while (opMode.opModeIsActive() & (timeout < 0 || waitTimer.time(TimeUnit.MILLISECONDS) < timeout)) {
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
        breakFollowing();

        follower.holdPoint(follower.getPose().setHeading(headingGoal));
        follower.update();

        while (opMode.opModeIsActive() && Math.abs(follower.getHeadingError()) > error) {
            update();
        }

        breakFollowing(false);
    }

    public void forceCancelShooting() {
        stopScoringCycle();
        shooterSubsystem.forceCloseFinger();
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public PathBuilder pathBuilder(PathConstraints pathConstraints) {
        return follower.pathBuilder(pathConstraints);
    }
}