package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited in the subsystem files

import com.pedropathing.VectorCalculator;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;
import org.firstinspires.ftc.teamcode.Base.Auto.RunSide;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.HardwareTable;
import org.firstinspires.ftc.teamcode.Base.Helpers.PedroUtils;
import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Misc.HeadingLockControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.HoodCompensationMethod;
import org.firstinspires.ftc.teamcode.Base.Misc.OpModeState;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterAimPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterPFState;
import org.firstinspires.ftc.teamcode.Base.Misc.ShootingStyle;
import org.firstinspires.ftc.teamcode.Base.Misc.TurretBacklashPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.TurretControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    // hardware is defined here

    private OpModeState currentState = OpModeState.GENERAL_CYCLE;
    private LinearOpMode opMode;
    private Follower follower = null;
    private VectorCalculator vectorCalculator = null;
    private Drivetrain drivetrain;
    private PoseTracker poseTracker;
    private AllianceSides side = Parameters.LAST_ALLIANCE_SIDE;
    private double autoTimeout = -1;
    private double goalOffset = 0;
    private AutoCommandRepository.AutoCommand lastAutoCommand = null;
    private boolean flagAutoPathingCancelled = false;
    private boolean autoRunBlockingEnabled = false;
    private Runnable autoCallback = null;
    private HardwareTable hardwareTable = new HardwareTable();

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

//    private Polygon closeZone = new Polygon();
//    private Polygon farZone = new Polygon();
//    private Polygon robotGeometricRepresentation;

    List<LynxModule> hubs = null;

    private ShooterControlPolicy shooterControlPolicy = ShooterControlPolicy.ROAMING;
    private ShooterAimPolicy shooterAimPolicy = ShooterAimPolicy.TURRET;
    private TurretControlPolicy turretControlPolicy = TurretControlPolicy.CONSTANT;
    private boolean turretEcoMode = true; // defaults to on, just like this was programmed by Ford.
    private boolean scoringCycleActive = false;
    private boolean scoringCycleOverrideWaitConditions = false;
    private boolean waitForVelocityToShoot = true;
    private boolean onlyShootInZone = false;
    private boolean shooterAimAtGoalActive = false;
    private boolean turretEnabled = false;
    private boolean turretRelativeControl = false;
    private double turretTargetPosition = Math.PI;
    private double hoodTargetPosition = 0;
    private double hoodTargetPositionOffset = 0;
    private TurretBacklashPolicy turretBacklashPolicy = TurretBacklashPolicy.MITIGATE_WHILE_SHOOTING;

    private double headingLockGoal = 0;
    private boolean headingLockActive = false;
    private boolean autoTeleShooting = false;
    private boolean followerEndStalePathEnabled = true;
    private double teleopHeadingOffset = 0;

    private double intakePower = 0;
    private double lastIntakePower = 0;
    private double transferSpeed = 1;
    private boolean activeHoldLastBallEnabled = true;
    private boolean activeHoldingLastBall = false;
    private final ElapsedTime activeHoldingLastBallTimer = new ElapsedTime();
    private final ElapsedTime shooterFingerTimeout = new ElapsedTime();

    private HeadingLockControlPolicy headingLockControlPolicy = HeadingLockControlPolicy.AIM_AT_GOAL;
    private boolean hoodCompensationEnabled = false;
    private HoodCompensationMethod hoodCompensationMethod = HoodCompensationMethod.CURRENT;
    private double hoodCompensationBaseCurrent = 0;
    private boolean velocityCompensationEnabled = true;
    private boolean centripetalVelocityCompensationEnabled = true;
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
    private boolean teleopDriveActive = false;

    // do NOT add a constructor to any of the subsystems!
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotManager(LinearOpMode newOpMode) {
        opMode = newOpMode;
        shooterSubsystem.setLinearTeleop(this.opMode);
        intakeSubsystem.setLinearTeleop(this.opMode);
    }

    public void setTurretBacklashPolicy(TurretBacklashPolicy newPolicy) {
        turretBacklashPolicy = newPolicy;
    }

    public TurretBacklashPolicy getTurretBacklashPolicy() {
        return turretBacklashPolicy;
    }

    public void setShooterControlPolicy(ShooterControlPolicy newControlPolicy) {
        shooterControlPolicy = newControlPolicy;
    }

    public void enableTurretEcoMode() {
        turretEcoMode = true;
    }

    public void disableTurretEcoMode() {
        turretEcoMode = false;
    }

    public ShooterControlPolicy getShooterControlPolicy() {
        return shooterControlPolicy;
    }

    public OpModeState getState() {
        return currentState;
    }

    public boolean isStateStart() {
        return stateStart;
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
        velocityCompensationEnabled = true;
    }

    public void disableVelocityCompensation() {
        velocityCompensationEnabled = false;
    }

    public void enableCentripetalVelocityCompensation() {
        centripetalVelocityCompensationEnabled = true;
    }

    public void disableCentripetalVelocityCompensation() {
        centripetalVelocityCompensationEnabled = false;
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

    public void setShooterAimPolicy(ShooterAimPolicy shooterAimPolicy) {
        this.shooterAimPolicy = shooterAimPolicy;
    }

    public ShooterAimPolicy getShooterAimPolicy() {
        return shooterAimPolicy;
    }

    public void initialise() {
        shooterSubsystem.initialiseHardware(hardwareTable);
        intakeSubsystem.initialiseHardware(hardwareTable);

        resetFollower();

        List<LynxModule> hubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        buildShooterCurves();
    }

    public void resetFollower() {
        follower = PedroConstants.getFollower(opMode.hardwareMap);
        poseTracker = follower.getPoseTracker();
        vectorCalculator = follower.getVectorCalculator();
        drivetrain = follower.getDrivetrain();
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
        Pose offsetedPose = getFixedPose(getGoalPosition(true, true).copy());

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
            Parameters.CLOSE_ZONE_CURVE.addPoint(40, 30, 3000);
            Parameters.CLOSE_ZONE_CURVE.addPoint(76, 100, 3680);
            Parameters.CLOSE_ZONE_CURVE.addPoint(83, 100, 3820);
            Parameters.CLOSE_ZONE_CURVE.addPoint(100, 120, 4350);

            Parameters.FAR_ZONE_CURVE.addPoint(160, 90, 5400);
            Parameters.FAR_ZONE_CURVE.addPoint(160, 90, 5450);

            shooterSubsystem.setHoodCompensationMultiplier(6);
        }

        Parameters.CLOSE_ZONE_CURVE.enableDomainLimit();
        Parameters.FAR_ZONE_CURVE.enableDomainLimit();

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

        if (!teleopDriveActive) {
            drivetrain.breakFollowing();
            teleopDriveActive = true;
        }

        Pose robotPose = getPose();
        Vector headingVector = new Vector();
        Vector driveVector = new Vector();

        if (headingLockActive) {
            double robotHeading = MathFunctions.normalizeAngle(robotPose.getHeading());
            double direction = MathFunctions.getTurnDirection(robotHeading, headingLockGoal);

            headingLockGoal = MathFunctions.normalizeAngle(headingLockGoal);

            headingVector = vectorCalculator.getHeadingVector(
                    MathFunctions.getSmallestAngleDifference(robotHeading, headingLockGoal) * direction,
                    robotPose,
                    headingLockGoal
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

        poseTracker.update();
        drivetrain.updateConstants();
        vectorCalculator.updateConstants();
    }

    public void enableHeadingLock() {
        headingLockActive = true;
        teleopDriveActive = true;
    }

    public void enableTurret() {
        if (!turretEnabled) {
            turretEnabled = true;

            shooterSubsystem.enableTurret();
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public void disableHeadingLock() {
        if (!shooterAimAtGoalActive || shooterAimPolicy == ShooterAimPolicy.TURRET) {
            headingLockActive = false;
        }
    }

    public void disableTurret() {
        if (turretEnabled) {
            turretEnabled = false;

            shooterSubsystem.enableTurret();
        }
    }

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    public void resetIMU() {
        if (!follower.isBusy()) {
            follower.setHeading(0);
        }
    }

    public void setHeadingLockControlPolicy(HeadingLockControlPolicy headingLockControlPolicy) {
        this.headingLockControlPolicy = headingLockControlPolicy;
    }

    public void setConstantHeadingLockGoal(double goal) {
        setHeadingLockControlPolicy(HeadingLockControlPolicy.CONSTANT);
        headingLockGoal = goal;
    }

    public void setConstantTurretHeadingGoal(double goal, AngleUnit angleUnit) {
        if (turretControlPolicy != TurretControlPolicy.CONSTANT) {
            turretControlPolicy = TurretControlPolicy.CONSTANT;
        }

        turretTargetPosition = goal;

        if (angleUnit == AngleUnit.DEGREES) {
            turretTargetPosition = Math.toRadians(turretTargetPosition);
        }
    }

    public void setConstantTurretHeadingGoal(double goal) {
        setConstantTurretHeadingGoal(goal, AngleUnit.DEGREES);
    }

    // This returns the turret target position, relative to the field.
    public double getTurretTargetPosition() {
        return turretTargetPosition;
    }

    public boolean turretCanReachTarget() {
        return shooterSubsystem.turretCanReachTarget();
    }

    public boolean isTurretRelativeControlEnabled() {
        return turretRelativeControl;
    }

    public void enableTurretRelativeControl() {
        if (getTurretControlPolicy() != TurretControlPolicy.CONSTANT) {
            turretControlPolicy = TurretControlPolicy.CONSTANT;
        }

        turretRelativeControl = true;
    }

    public void disableTurretRelativeControl() {
        turretRelativeControl = false;
    }

    public void powerShooterOn() {
        shooterSubsystem.powerOn();
    }

    public void powerIntakeOff() {
        setIntakePower(0);
    }

    public void powerIntakeOn() {
        setIntakePower(1);
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

    public void powerShooterOff() {
        shooterSubsystem.powerOff();
    }

    public void startScoringCycle(boolean overrideWaitConditions) {
        canShoot = false;
        canShootOveride = false;
        scoringCycleActive = true;
        scoringCycleOverrideWaitConditions = overrideWaitConditions;
        shooterFingerTimeout.reset();

        if (hoodCompensationEnabled && hoodCompensationMethod == HoodCompensationMethod.CURRENT) {
            hoodCompensationBaseCurrent = shooterSubsystem.getShooterCurrent();
        }
    }

    public void startScoringCycle() {
        startScoringCycle(false);
    }

    public void stopScoringCycle() {
        scoringCycleActive = false;
    }

    public void enableAutoShooterControl() {
        shooterControlPolicy = ShooterControlPolicy.ROAMING;
    }

    public void enableManualShooterControl() {
        shooterControlPolicy = ShooterControlPolicy.MANUAL;
    }

    public boolean scoringCycleActive() {
        return scoringCycleActive;
    }

    public void setState(OpModeState newState) {
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
        if (shooterControlPolicy == ShooterControlPolicy.MANUAL) {
            hoodTargetPosition = newPos;
        }
    }

    public Pose getPose() {
        return poseTracker.getPose();
    }

    public void setShootingStyle(ShootingStyle shootingStyle) {
        this.shootingStyle = shootingStyle;

        // rebuild the curves if the driver wants to change how they shoot mid-match
        buildShooterCurves();
    }

    public ShootingStyle getShootingStyle() {
        return this.shootingStyle;
    }

    public void startAimingAtGoal() {
        if (shooterAimPolicy == ShooterAimPolicy.TURRET) {
            turretControlPolicy = TurretControlPolicy.AIM_AT_GOAL;
            enableTurret();
        } else if (shooterAimPolicy == ShooterAimPolicy.DRIVETRAIN) {
            setHeadingLockControlPolicy(HeadingLockControlPolicy.AIM_AT_GOAL);
            enableHeadingLock();
        }
    }

    public void stopAimingAtGoal() {
        shooterAimAtGoalActive = false;

        if (shooterAimPolicy == ShooterAimPolicy.TURRET) {
            turretControlPolicy = TurretControlPolicy.CONSTANT;
        } else if (shooterAimPolicy == ShooterAimPolicy.DRIVETRAIN) {
            disableHeadingLock();
        }
    }

    public boolean isAimingAtGoal() {
        return shooterAimAtGoalActive;
    }

    private void internalRunPath(PathBuilder path, boolean correctAfterFinished) {
        breakFollowing();

        autoRunBlockingEnabled = true;

        follower.followPath(path.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive() && !flagAutoPathingCancelled) {
            if (autoTimeout > 0 && autoTimer.time(TimeUnit.MILLISECONDS) >= autoTimeout) {
                breakFollowing(correctAfterFinished);
                break;
            }

            if (followerEndStalePathEnabled) {
                if (follower.isRobotStuck()) {
                    breakFollowing(correctAfterFinished);
                }
            }

            update();
            follower.update();
        }

        clearPathTimeout();
        autoRunBlockingEnabled = false;
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
     * This will return a copy of the inputted heading value that could be mirrored if the robot
     * is on blue.
     */
    public double getFixedHeading(double heading, AngleUnit angleUnit) {
        double correctedHeading = heading;

        if (angleUnit == AngleUnit.DEGREES) {
            correctedHeading = Math.toRadians(heading);
        }

        if (side == AllianceSides.RED) {
            return correctedHeading; // no mirroring is needed
        } else {
            return PedroUtils.getMirroredPose(new Pose(0, 0, correctedHeading)).getHeading();
        }
    }

    /*
     * This will return a copy of the inputted heading value (in degrees) that could be mirrored if the robot
     * is on blue.
     */
    public double getFixedHeading(double heading) {
        return getFixedHeading(heading, AngleUnit.DEGREES);
    }

    public void runBlocking(PathBuilder path, boolean correctAfterFinished) {
        if (!follower.isBusy()) internalRunPath(path, correctAfterFinished);
    }

    public void safeSleep(double time) {
        ElapsedTime timer = new ElapsedTime();

        safeSleep(timer, time);
    }

    public void safeSleep(ElapsedTime timer, double time) {
        waitForCondition(() -> timer.time(TimeUnit.MILLISECONDS) >= time);
    }

    // The robot will stop waiting once the condition returns `true`
    public void waitForCondition(Supplier<Boolean> condition) {
        while (!condition.get() && opMode.opModeIsActive() && !flagAutoPathingCancelled) {
            update();
            follower.update();
        }
    }

    public void waitForPathEnd() {
        waitForCondition(() -> !follower.isBusy());
    }

    public void addPathTimeout(double timeout) {
        autoTimeout = timeout;
        autoTimer.reset();
    }

    public void clearPathTimeout() {
        autoTimeout = -1;
        autoTimer.reset();
    }

    public Pose getVelocityCorrectedPose(Pose robotPose, Vector velocity) {
        return robotPose
                .plus(
                        new Pose(
                                velocity.getXComponent() * Parameters.VELOCITY_CORRECTION_MULTIPLIER,
                                velocity.getYComponent() * Parameters.VELOCITY_CORRECTION_MULTIPLIER
                        )
                );
    }

    public Pose getVelocityCorrectedPose() {
        return getVelocityCorrectedPose(getPose(), poseTracker.getVelocity());
    }

    public double getHeadingToGoal(Pose robotPose) {
        return getHeadingToPose(robotPose, getOffsetedGoalPose());
    }

    public double getHeadingToPose(Pose robotPose, Pose targetPose) {
        return (Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()) + Math.PI);
    }

    public void stopAndAim() {
        breakFollowing(false);

        Pose holdPose = follower.getPose();

        if (shooterAimPolicy == ShooterAimPolicy.DRIVETRAIN) {
            holdPose = holdPose.setHeading(getHeadingToGoal(holdPose));
        }

        follower.holdPoint(holdPose);
    }

    public void enableOnlyShootInZone() {
        onlyShootInZone = true;
    }

    public void disableOnlyShootInZone() {
        onlyShootInZone = false;
    }

    public double getHeadingToGoal() {
        return getHeadingToGoal(velocityCompensationEnabled ? getVelocityCorrectedPose() : getPose());
    }

    public void recalibrateIMU() {
        if (PedroConstants.pinpointLocalizer != null) {
            PedroConstants.pinpointLocalizer.resetIMU();
            PedroConstants.pinpointLocalizer.recalibrate();
            PedroConstants.pinpointLocalizer.resetIMU();
        }
    }

    public void setTransferSpeed(double transferSpeed) {
        this.transferSpeed = transferSpeed;
    }

    public void enableHoodCompensation() {
        hoodCompensationEnabled = true;
    }

    public void disableHoodCompensation() {
        hoodCompensationEnabled = false;
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
            hoodTargetPosition = Parameters.CLOSE_ZONE_CURVE.getHoodCurveOutput(distanceToGoalInput);
        } else {
            shooterSubsystem.setVelocity(Parameters.SHOOTER_FAR_ZONE_VELOCITY);
            hoodTargetPosition = Parameters.SHOOTER_FAR_ZONE_HOOD_ANGLE;
        }
    }

    public void updateShooterParameters(Pose robotPose) {
        updateShooterParameters(getDistanceToGoal(robotPose));
    }

    public void runSingleAutoCommand(AutoCommandRepository.AutoCommand command, RunSide runSide) {
        resetAutoPathingCancelFlag();

        command.execute(
                this,
                new Pose(),
                runSide,
                lastAutoCommand,
                null
        );

        lastAutoCommand = command;
    }

    public void cancelAutoPathing() {
        flagAutoPathingCancelled = true;

        if (!autoRunBlockingEnabled && follower.isBusy()) {
            // runPassthrough was called.
            follower.breakFollowing();
        }
    }

    public boolean autoPathingCancelFlagActive() {
        return flagAutoPathingCancelled;
    }

    public void resetAutoPathingCancelFlag() {
        flagAutoPathingCancelled = false;
    }

    public void registerAutoCallback(Runnable autoCallback) {
        this.autoCallback = autoCallback;
    }

    public void clearAutoCallback() {
        this.autoCallback = null;
    }

    public void setHoodCompensationMethod(HoodCompensationMethod method) {
        this.hoodCompensationMethod = method;
    }

    public void update() {
        Pose robotPose = getPose();

        if (follower.isBusy() && autoCallback != null) {
            autoCallback.run();
        }

        if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
            return;
        }

        if (timer.time(TimeUnit.MILLISECONDS) % 5 == 0) {
            clearCache();
        }

        double headingToGoal = getHeadingToGoal(velocityCompensationEnabled ? getVelocityCorrectedPose(robotPose, poseTracker.getVelocity()) : robotPose);

        if (headingLockControlPolicy == HeadingLockControlPolicy.AIM_AT_GOAL) {
            headingLockGoal = headingToGoal;
        }

        if (!turretEcoMode || (shooterSubsystem.isPoweredOn() || scoringCycleActive)) {
            if (turretEnabled) {
                if (turretControlPolicy == TurretControlPolicy.AIM_AT_GOAL) {
                    turretTargetPosition = headingToGoal - Math.PI;
                }

                double turretPositionCorrected = turretTargetPosition;

                if (!turretRelativeControl) {
                    turretPositionCorrected -= robotPose.getHeading();

                    if (centripetalVelocityCompensationEnabled) {
                        turretPositionCorrected -= (poseTracker.getAngularVelocity() * Parameters.CENTRIPETAL_VELOCITY_COMPENSATION_MULTIPLIER);
                    }
                }

                shooterSubsystem.setTurretPosition(turretPositionCorrected);
            }
        } else {
            shooterSubsystem.disableTurret();
        }

        if (shooterControlPolicy == ShooterControlPolicy.ROAMING) {
            updateShooterParameters(getDistanceToGoal());
        }

        if (turretBacklashPolicy == TurretBacklashPolicy.MITIGATE_ALWAYS || (scoringCycleActive && turretBacklashPolicy == TurretBacklashPolicy.MITIGATE_WHILE_SHOOTING)) {
            shooterSubsystem.setTurretBacklashOffset(Parameters.TURRET_BACKLASH);
        } else {
            shooterSubsystem.clearTurretBacklashOffset();
        }

        if (scoringCycleActive) {
            boolean needsToWait = false;

            isTransferStopped = false;
            transferStopTimerPaused = true;

            if (hoodCompensationEnabled) {
                if (hoodCompensationMethod == HoodCompensationMethod.CURRENT) {
                    hoodTargetPosition = shooterSubsystem.getShooterCurrent() - hoodCompensationBaseCurrent * Parameters.HOOD_COMPENSATION_CURRENT_MULTIPLIER;
                } else if (hoodCompensationMethod == HoodCompensationMethod.VELOCITY) {
                    hoodTargetPosition = (shooterSubsystem.getTargetVelocity() - shooterSubsystem.getVelocity()) * Parameters.HOOD_COMPENSATION_VELOCITY_MULTIPLIER;
                }

                hoodTargetPosition = Range.clip(hoodTargetPosition, 0, Parameters.HOOD_COMPENSATION_MAX_OFFSET);
            }

            if (waitForVelocityToShoot) {
                if (!shooterSubsystem.ready() && !canShootOveride) {
                    needsToWait = true;
                }
            }

            if (shooterAimPolicy == ShooterAimPolicy.TURRET && !turretCanReachTarget()) {
                needsToWait = true;
            }

            activeHoldingLastBall = false;

            intakeSubsystem.setIntakePower(intakePower);

            if (!needsToWait || scoringCycleOverrideWaitConditions) {
                if (!canShoot) {
                    canShoot = true;
                    canShootAtVelocity = shooterSubsystem.getTargetVelocity();
                }

                if (intakeSubsystem.getIntakePower() >= .1) {
                    canShootOveride = true;
                    double distanceToGoal = getDistanceToGoal(robotPose); // ignore velocity correction

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

                if (intakePower >= 0) {
                    if (autoTransferStopEnabled) {
                        if (isTransferStopped) {
                            intakeMotor2Power = 0;
                        } else {
                            if (intakeSubsystem.isTransferOverCurrent(CurrentUnit.AMPS, Parameters.TRANSFER_STOP_CURRENT)) {
                                if (transferStopTimerPaused) {
                                    transferStopTimer.reset();
                                    transferStopTimerPaused = false;
                                }

                                if (transferStopTimer.time(TimeUnit.MILLISECONDS) >= 400) {
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

            if (shooterFingerTimeout.time(TimeUnit.MILLISECONDS) >= Parameters.SHOOTER_FINGER_TIMEOUT) {
                shooterSubsystem.closeFinger();
            }
        }

        if (printDebugEnabled) {
            printDebugInfo();
        }

        stateStart = false;
        resetDistanceToGoal = true;

        if (!teleopDriveActive) {
            follower.update();
        }

        shooterSubsystem.setHoodPos(hoodTargetPosition - hoodTargetPositionOffset);

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

    public double getDriverOffset() {
        return teleopHeadingOffset;
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

    public Double getShooterVelocity() {
        return shooterSubsystem.getVelocity();
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
        teleopDriveActive = false;

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

    public Pose getGoalPosition(boolean checkDistance, boolean retrieveAimPosition) {
        Pose robotPose = new Pose();

        if (checkDistance)
            robotPose = getPose();

        if (getAllianceSide() == AllianceSides.RED) {
            if (!checkDistance || robotPose.distanceFrom(Parameters.SHOOTER_GOAL_CLOSE_RED) < Parameters.FAR_ZONE_DISTANCE) {
                return retrieveAimPosition ? Parameters.SHOOTER_GOAL_CLOSE_RED_AIM : Parameters.SHOOTER_GOAL_CLOSE_RED;
            } else {
                return retrieveAimPosition ? Parameters.SHOOTER_GOAL_FAR_RED_AIM : Parameters.SHOOTER_GOAL_FAR_RED;
            }
        } else {
            if (!checkDistance || robotPose.distanceFrom(Parameters.SHOOTER_GOAL_CLOSE_BLUE) < Parameters.FAR_ZONE_DISTANCE) {
                return retrieveAimPosition ? Parameters.SHOOTER_GOAL_CLOSE_BLUE_AIM : Parameters.SHOOTER_GOAL_CLOSE_BLUE;
            } else {
                return retrieveAimPosition ? Parameters.SHOOTER_GOAL_FAR_BLUE_AIM : Parameters.SHOOTER_GOAL_FAR_BLUE;
            }
        }
    }

    public Pose getGoalPosition() {
        return getGoalPosition(true, false);
    }

    public double getDistanceToGoal() {
        if (resetDistanceToGoal) {
            Pose robotPose = velocityCompensationEnabled ? getVelocityCorrectedPose() : getPose();

            resetDistanceToGoal = false;
            distanceToGoal = robotPose.distanceFrom(getGoalPosition());
        }
        return distanceToGoal;
    }

    public double getDistanceToGoal(Pose robotPosition) {
        return robotPosition.distanceFrom(getGoalPosition(false, false));
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

    public TurretControlPolicy getTurretControlPolicy() {
        return turretControlPolicy;
    }

    public boolean isFollowerBusy() {
        return !teleopDriveActive && follower.isBusy();
    }

    public ArrayList<ComplexServo> getServos() {
        return hardwareTable.getServos();
    }

    public ArrayList<ComplexMotor> getMotors() {
        return hardwareTable.getMotors();
    }
}