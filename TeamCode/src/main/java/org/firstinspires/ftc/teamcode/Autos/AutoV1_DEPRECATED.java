//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Base.AllianceSides;
//import org.firstinspires.ftc.teamcode.Base.OpModeStates;
//import org.firstinspires.ftc.teamcode.Base.Parameters;
//import org.firstinspires.ftc.teamcode.Base.RobotManager;
//import org.firstinspires.ftc.teamcode.Base.ShooterControlPolicy;
//import org.firstinspires.ftc.teamcode.Base.ShootingStyle;
//import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
//import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;
//import org.firstinspires.ftc.teamcode.bedroBathing.util.CustomPIDFCoefficients;
//
//import java.util.ArrayList;
//import java.util.concurrent.TimeUnit;
//
//enum AutoStartPos {
//    CLOSE_ZONE,
//    FAR_ZONE
//}
//
//enum AutoType {
//    FIFTEEN_ARTIFACT,
//    FIFTEEN_ALLIANCE_FRIENDLY,
//    TWELVE_ARTIFACT,
//    EIGHTEEN_ARTIFACT,
//    EIGHTEEN_ALLIANCE_FRIENDLY,
//    CLOSE_ZONE_9,
//    CLOSE_ZONE_6,
//    GATE_INTAKE_DEBUG,
//}
//
//@Autonomous(name = "DEPRECATED Auto V1", group = "1", preselectTeleOp = "0: Main Teleop")
//public class AutoV1_DEPRECATED extends LinearOpMode {
//    private RobotManager robot;
//    private AutoStartPos autoStartPos = AutoStartPos.CLOSE_ZONE;
//    private boolean clearGate = true;
//    private boolean grabFromLine0 = true;
//    private boolean grabFromHumanPlayer = true;
//    private int grabFromGateCycles = 0;
//    private boolean delayedSecondGateCycle = false;
//    private boolean farGrabFromLine = true;
//    private boolean robotStartIsSet = false;
//    private boolean shootLastCycleOffTape = false;
//    private boolean debugPark = false;
//    private AutoType autoType = AutoType.EIGHTEEN_ARTIFACT;
//
//    private ElapsedTime timer = new ElapsedTime();
//    private ArrayList<String> headingErrors = new ArrayList<>();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new RobotManager(this);
//
//        robot.setState(OpModeStates.INTAKE_SCORE);
//        robot.setTransferSpeed(1);
//        robot.setShooterControlPolicy(ShooterControlPolicy.MANUAL);
//        robot.enableAutoTransferStop();
//        robot.disableDebugPrinting();
//        robot.disableVelocityCompensation();
//        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
//        robot.disableHoodCompensation();
//        robot.disableOnlyShootInZone();
//        robot.disableWaitForVelocityToShoot();
//        robot.disablePoweredHold();
//
//        robot.initialise();
//        robot.recalibrateIMU();
//
//        robot.startScoringCycle();
//        robot.update();
//        robot.update();
//        robot.update();
//
//        int autoTypeSelection = autoType.ordinal();
//
//        while (opModeInInit()) {
//            robot.powerOffShooter();
//
//            if (gamepad1.yWasPressed())
//                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.BLUE ? AllianceSides.RED : AllianceSides.BLUE);
//
//            telemetry.addLine("Press Δ to change the robot's alliance.");
//            telemetry.addLine("Press DPad Up/Down to change Autonomous Mode.");
//
//            if (gamepad1.dpadUpWasPressed()) {
//                autoTypeSelection++;
//            } else if (gamepad1.dpadDownWasPressed()) {
//                autoTypeSelection--;
//            }
//
//            if (autoTypeSelection >= AutoType.values().length) {
//                autoTypeSelection = 0;
//            } else if (autoTypeSelection < 0) {
//                autoTypeSelection = (AutoType.values().length - 1);
//            }
//
//            autoType = AutoType.values()[autoTypeSelection];
//            telemetry.addData("Alliance Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
//            telemetry.addData("Autonomous Mode: ", autoType);
//            telemetry.addData("Robot Heading: ", robot.getPose().getHeading());
//
//            updateAutoSettings();
//            updateRobotStart();
//
//            robot.update();
//            telemetry.update();
//        }
//
//        updateAutoSettings();
//        updateRobotStart();
//
//        robot.enableDebugPrinting();
//
//        waitForStart();
//
//        int gateCycleNumber = 0;
//
//        robot.update();
//        robot.getPose();
//
//        timer.reset();
//        Parameters.LAST_ALLIANCE_SIDE = robot.getAllianceSide();
//
//        shootBalls(0, false, false);
//
//        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
//            if (grabFromGateCycles == 0) {
//                if (grabFromLine0) {
//                    intakeFromTape(0);
//                    shootBalls(1, false, false);
//                }
//                intakeFromTape(1);
//                shootBalls(2, false, false);
//                intakeFromTape(2);
//                shootBalls(3, false, false);
//
//                if (grabFromHumanPlayer && clearGate) {
//                    intakeFromHumanPlayer();
//                    shootBalls(3, false, false);
//                }
//            } else if (grabFromGateCycles > 0) {
//                intakeFromTape(1);
//                shootBalls(2, false, false);
//
//                if (delayedSecondGateCycle) {
//                    intakeFromGate(gateCycleNumber++);
//                    shootBalls(3, true, false);
//
//                    grabFromGateCycles--;
//                } else {
//                    for (int i = 0; i < grabFromGateCycles; i++) {
//                        intakeFromGate(gateCycleNumber++);
//                        shootBalls(3, true, false);
//                    }
//                }
//
//                if (grabFromLine0) {
//                    intakeFromTape(0);
//                    shootBalls(1, false, false);
//                }
//
//                if (grabFromHumanPlayer) {
//                    intakeFromHumanPlayer();
//                    shootBalls(3, false, false);
//                }
//
//                intakeFromTape(2);
//
//                if (clearGate || grabFromGateCycles != 0) {
//                    shootBalls(3, false, shootLastCycleOffTape && !delayedSecondGateCycle);
//                }
//
//                if (delayedSecondGateCycle && grabFromGateCycles > 0) {
//                    for (int i = 0; i < grabFromGateCycles; i++) {
//                        intakeFromGate(gateCycleNumber++);
//                        shootBalls(3, true, shootLastCycleOffTape);
//                    }
//                }
//            }
//
//            if (!shootLastCycleOffTape)
//                park();
//
//            if (debugPark) {
//                robot.powerOffShooter();
//                robot.powerOffIntake();
//
//                robot.safeSleep(2000);
//
//                Pose robotPose = robot.getPose();
//                Pose startPose = getStartPose();
//
//                robot.setMaxFollowerPower(.5);
//                robot.runBlocking(
//                        new PathBuilder()
//                                .addPath(new Path(
//                                        new BezierLine(
//                                                new Point(robotPose),
//                                                new Point(startPose)
//                                        )
//                                ))
//                                .addLinearHeadingInterpolation(0, robotPose.getHeading(), startPose.getHeading(), 1)
//                                .setPathEndTValueConstraint(1)
//                                .setZeroPowerAccelerationMultiplier(4)
//                                .setPathEndTValueConstraint(.92)
//                );
//            }
//        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
//            int shootOffset = 0;
//
//            if (farGrabFromLine) {
//                shootOffset = 1;
//
//                intakeFromTape(0);
//                shootBalls(1, false, false);
//            }
//
//            intakeFromHumanPlayer();
//            shootBalls(1 + shootOffset, false, false);
//
//            robot.powerOffShooter();
//            robot.powerOffIntake();
//
//            while (opModeIsActive() && timer.time(TimeUnit.SECONDS) <= 27) {
//                robot.update();
//            }
//
//            park();
//        }
//
//        robot.powerOffShooter();
//        robot.powerOffIntake();
//        robot.update();
//
//        double time = ((double) timer.time(TimeUnit.MILLISECONDS)) / 1000;
//
//        while (opModeIsActive()) {
//            telemetry.addData("Time: ", time);
//            telemetry.addData("Heading Errors: ", String.join(", ", headingErrors));
//            telemetry.update();
//        }
//
//        Parameters.OPMODE_END_POSITION = robot.getPose();
//    }
//
//    private Pose getStartPose() {
//        Pose startPose = new Pose();
//
//        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
//            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START;
//        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
//            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START;
//        }
//
//        return startPose;
//    }
//
//    private void updateRobotStart() {
//        if (opModeIsActive()) {
//            if (robotStartIsSet) return;
//            robotStartIsSet = true;
//        } else {
//            robotStartIsSet = false;
//        }
//
//        robot.setPose(getStartPose());
//    }
//
//    private void updateAutoSettings() {
//        switch (autoType) {
//            case TWELVE_ARTIFACT:
//                clearGate = true;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = false;
//                grabFromGateCycles = 0;
//                delayedSecondGateCycle = false;
//                grabFromLine0 = true;
//                shootLastCycleOffTape = false;
//                debugPark = false;
//                break;
//            case FIFTEEN_ARTIFACT:
//                clearGate = true;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = true;
//                grabFromGateCycles = 0;
//                delayedSecondGateCycle = false;
//                grabFromLine0 = true;
//                shootLastCycleOffTape = false;
//                debugPark = false;
//                break;
//            case FIFTEEN_ALLIANCE_FRIENDLY:
//                clearGate = false;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = false;
//                grabFromGateCycles = 2;
//                delayedSecondGateCycle = true;
//                grabFromLine0 = false;
//                shootLastCycleOffTape = false;
//                debugPark = false;
//                break;
//            case EIGHTEEN_ARTIFACT:
//                clearGate = false;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = false;
//                grabFromGateCycles = 2;
//                delayedSecondGateCycle = false;
//                grabFromLine0 = true;
//                shootLastCycleOffTape = true;
//                debugPark = false;
//                break;
//            case EIGHTEEN_ALLIANCE_FRIENDLY:
//                clearGate = false;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = false;
//                grabFromGateCycles = 3;
//                delayedSecondGateCycle = false;
//                grabFromLine0 = false;
//                shootLastCycleOffTape = true;
//                debugPark = false;
//                break;
//            case GATE_INTAKE_DEBUG:
//                clearGate = false;
//                autoStartPos = AutoStartPos.CLOSE_ZONE;
//                grabFromHumanPlayer = false;
//                grabFromGateCycles = 6;
//                delayedSecondGateCycle = false;
//                grabFromLine0 = false;
//                shootLastCycleOffTape = true;
//                debugPark = true;
//                break;
//            case CLOSE_ZONE_9:
//                clearGate = false;
//                autoStartPos = AutoStartPos.FAR_ZONE;
//                grabFromHumanPlayer = true;
//                grabFromGateCycles = 0;
//                delayedSecondGateCycle = false;
//                farGrabFromLine = true;
//                shootLastCycleOffTape = true;
//                debugPark = false;
//                break;
//            case CLOSE_ZONE_6:
//                clearGate = false;
//                autoStartPos = AutoStartPos.FAR_ZONE;
//                grabFromHumanPlayer = true;
//                grabFromGateCycles = 0;
//                delayedSecondGateCycle = false;
//                farGrabFromLine = false;
//                shootLastCycleOffTape = true;
//                debugPark = false;
//                break;
//        }
//    }
//
//    // 0 is the line furthest from the goal
//    private void intakeFromTape(double number) {
//        robot.stopScoringCycle();
//        robot.setIntakePower(1);
//        robot.setMaxFollowerPower(1);
//        robot.update();
//
//        Pose robotPose = robot.getPose();
//
//        if (number == 0) {
//            robot.addPathTimeout(2000);
//            robot.runBlocking(new PathBuilder()
//                            .addPath(new Path(
//                                    new BezierCurve(
//                                            new Point(robotPose),
//                                            robot.getFixedPoint(-25, -102),
//                                            robot.getFixedPoint(7, -98)
//                                    )
//                            ))
//                            .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(0), .7)
//                            .addLinearHeadingInterpolation(.7, robot.getFixedHeading(0), robot.getFixedHeading(0), 1)
//                            .setPathEndTValueConstraint(.99)
//                            .setPathEndVelocityConstraint(1000)
//                            .setZeroPowerAccelerationMultiplier(10)
//                    , true);
//        } else if (number == 1) {
//            robot.addPathTimeout(2000);
//            robot.runBlocking(new PathBuilder()
//                            .addPath(new Path(
//                                    new BezierCurve(
//                                            new Point(robotPose),
//                                            robot.getFixedPoint(-25, -76),
//                                            robot.getFixedPoint(10, -74)
//                                    )
//                            ))
//                            .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(0), .7)
//                            .addLinearHeadingInterpolation(.7, robot.getFixedHeading(0), robot.getFixedHeading(0), 1)
//                            .setPathEndTValueConstraint(.99)
//                            .setPathEndVelocityConstraint(1000)
//                            .setZeroPowerAccelerationMultiplier(10)
//                    , true);
//        } else if (number == 2) {
//            robot.turnTo(robot.getFixedHeading(0), Math.toRadians(35));
//
//            robotPose = robot.getPose();
//
//            robot.setMaxFollowerPower(.9);
//
//            robot.addPathTimeout(2000);
//            robot.runBlocking(new PathBuilder()
//                            .addPath(new Path(
//                                    new BezierLine(
//                                            new Point(robotPose),
//                                            robot.getFixedPoint(4, -50)
//                                    )
//                            ))
//                            .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(0), .4)
//                            .addLinearHeadingInterpolation(.4, robot.getFixedHeading(0), robot.getFixedHeading(0), 1)
//                            .setPathEndTValueConstraint(.99)
//                            .setPathEndVelocityConstraint(1000)
//                            .setZeroPowerAccelerationMultiplier(10)
//                    , true);
//
//            robot.setMaxFollowerPower(1);
//        }
//
//        if (number == 1) {
//            if (clearGate) {
//                clearGate();
//            }
//
//            return;
//        }
//
//        robot.setMaxFollowerPower(1);
//    }
//
//    private void intakeFromGate(int cycleNumber) {
//        Pose robotPose = robot.getPose();
//
//        double gateHeading = 25;
//        double gateX = 13;
//        double gateY = -75;
//
//        robot.setMaxFollowerPower(1);
//        robot.setIntakePower(1);
//        robot.addPathTimeout(1750);
//        robot.runBlocking(
//                new PathBuilder()
//                        .addPath(new Path(
//                                new BezierCurve(
//                                        new Point(robotPose),
//                                        robot.getFixedPoint(-9, -78),
//                                        robot.getFixedPoint(gateX, gateY)
//                                )
//                        ))
//                        .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(gateHeading), .3)
//                        .addLinearHeadingInterpolation(.3, robot.getFixedHeading(gateHeading), robot.getFixedHeading(gateHeading), 1)
//                        .setPathEndTValueConstraint(.94)
//                        .setZeroPowerAccelerationMultiplier(2)
//                        .setPathEndHeadingConstraint(Math.PI)
//                        .setPathEndVelocityConstraint(1000)
//                        .setPathEndTranslationalConstraint(1000)
//        , false);
//
//        robot.breakFollowing(false);
//        robot.setMaxFollowerPower(1);
//
//        robotPose = robot.getPose();
//        robot.clearPathTimeout();
//        robot.runPassthrough(
//                new PathBuilder()
//                        .addPath(new Path(
//                                new BezierLine(
//                                        new Point(robotPose),
//                                        robot.getFixedPoint(17, gateY)
//                                )
//                        ))
//                        .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(gateHeading), 1)
//                        .setPathEndTValueConstraint(1)
//                        .setZeroPowerAccelerationMultiplier(3));
//
//        robot.safeSleep(1250);
//        robot.breakFollowing(false);
//        robot.clearPathTimeout();
//    }
//
//    private void clearGate() {
//        Pose robotPose = robot.getPose();
//
//        double pushHeading = robot.getFixedHeading(0);
//
//        robot.powerOffIntake();
//        robot.setMaxFollowerPower(1);
//        robot.addPathTimeout(900);
//
//        boolean oldValue = FollowerConstants.useSecondaryHeadingPID;
//        FollowerConstants.useSecondaryHeadingPID = false;
//        ElapsedTime timer = new ElapsedTime();
//
//        timer.reset();
//
//        robot.runBlocking(new PathBuilder()
//                        .addPath(new Path(
//                                new BezierCurve(
//                                        new Point(robotPose),
//                                        robot.getFixedPoint(-6, -70),
//                                        robot.getFixedPoint(9, -59)
//                                )
//                        ))
//                        .addLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
//                        .setPathEndTValueConstraint(.99)
//                        .setZeroPowerAccelerationMultiplier(5)
//                , false);
//
//        robotPose = robot.getPose();
//
//        robot.setMaxFollowerPower(.3);
//        robot.runPassthrough(new PathBuilder()
//                        .addPath(new Path(
//                                new BezierLine(
//                                        new Point(robotPose),
//                                        robot.getFixedPoint(18, -59)
//                                )
//                        ))
//                        .addLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
//                        .setPathEndTValueConstraint(.99)
//                        .setZeroPowerAccelerationMultiplier(5));
//
//        FollowerConstants.useSecondaryHeadingPID = oldValue;
//
//        robot.safeSleep(timer, 2500);
//        robot.breakFollowing();
//        robot.setMaxFollowerPower(1);
//    }
//
//    private void intakeFromHumanPlayer() {
//        Pose robotPose = robot.getPose();
//
//        robot.stopScoringCycle();
//        robot.setIntakePower(1);
//        robot.setMaxFollowerPower(1);
//        robot.update();
//
//        final double wallY = 14.5; // 14.2
//        double endHeading = robot.getFixedHeading(280);
//
//        robot.setMaxFollowerPower(.8);
//
//        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
//            robot.runBlocking(new PathBuilder()
//                    .addPath(new Path(
//                            new BezierCurve(
//                                    new Point(robotPose),
//                                    robot.getFixedPoint(-40, -80),
//                                    robot.getFixedPoint(wallY, -94)
//                            )
//                    ))
//                    .addTangentHeadingInterpolation(0, false, .5)
//                    .addConstantHeadingInterpolation(.5, endHeading, 1)
//                    .setPathEndTValueConstraint(.9)
//                    .setPathEndVelocityConstraint(100)
//                    .setPathEndTimeoutConstraint(0)
//                    .setZeroPowerAccelerationMultiplier(15), true);
//        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
//            robot.runBlocking(new PathBuilder()
//                    .addPath(new Path(
//                            new BezierLine(
//                                    new Point(robotPose),
//                                    robot.getFixedPoint(wallY, -94)
//                            )
//                    ))
//                    .addLinearHeadingInterpolation(0, robotPose.getHeading(), endHeading, 1)
//                    .setPathEndTValueConstraint(.9)
//                    .setPathEndVelocityConstraint(100)
//                    .setPathEndTimeoutConstraint(0)
//                    .setZeroPowerAccelerationMultiplier(15), true);
//        }
//
//        robot.setMaxFollowerPower(.95);
//        robot.addPathTimeout(1500);
//        robot.runBlocking(new PathBuilder()
//                .addPath(new Path(
//                        new BezierLine(
//                                robot.getFixedPoint(wallY, -94),
//                                robot.getFixedPoint(wallY, -118)
//                        )
//                ))
//                .addLinearHeadingInterpolation(0, robot.getPose().getHeading(), endHeading, 1)
//                .setPathEndTValueConstraint(.95)
//                .setPathEndVelocityConstraint(100)
//                .setZeroPowerAccelerationMultiplier(4.5), false);
//
//        robot.safeSleep(750);
//        robot.powerOffIntake();
//    }
//
//    private void shootBalls(double cycleNumber, boolean afterGate, boolean shootOffTape) {
//        Pose shootingPosition;
//        Pose robotPose = robot.getPose();
//
//        robot.powerOnShooter();
//        robot.setMaxFollowerPower(1);
//
//        CustomPIDFCoefficients oldSecondaryHeading = FollowerConstants.secondaryHeadingPIDFCoefficients;
//        double oldHoldPointScaling = FollowerConstants.holdPointHeadingScaling;
//        boolean oldUseSecondaryHeading = FollowerConstants.useSecondaryHeadingPID;
//        double oldHeadingPIDFSwitch = FollowerConstants.headingPIDFSwitch;
//        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients = Parameters.preciseTurnCoeffs;
//        FollowerConstants.holdPointHeadingScaling = 1;
//        FollowerConstants.headingPIDFSwitch = Math.PI / 20;
//
//        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
//            if (cycleNumber == 0) {
//                robot.startScoringCycle(); // make sure that servo opens!
//                robot.update();
//
//                shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
//                robot.updateShooterParameters(shootingPosition);
//
//                robot.addPathTimeout(2000);
//                robot.runBlocking(new PathBuilder()
//                                .addPath(new Path(
//                                        new BezierLine(
//                                                new Point(getStartPose()),
//                                                new Point(shootingPosition)
//                                        )
//                                ))
//                                .addVariableHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading(), robot::getHeadingToGoal)
//                                .setPathEndTValueConstraint(.9)
//                                .setPathEndVelocityConstraint(5)
//                                .setZeroPowerAccelerationMultiplier(4)
//                        , false);
//            } else {
//                if (shootOffTape) {
//                    shootingPosition = robot.getFixedPose(-28, -28, Math.toRadians(180));
//                } else {
//                    shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
//                }
//
//                robot.updateShooterParameters(shootingPosition);
//
//                if (cycleNumber == 1) {
//                    robot.runBlocking(new PathBuilder()
//                                    .addPath(new Path(
//                                            new BezierCurve(
//                                                    new Point(robotPose),
//                                                    robot.getFixedPoint(-20, robotPose.getY() - 15),
//                                                    new Point(shootingPosition)
//                                            )
//                                    ))
//                                    .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
//                                    .addParametricCallback(.3, () -> robot.powerOffIntake())
//                                    .setPathEndTValueConstraint(.95)
//                                    .setZeroPowerAccelerationMultiplier(6)
//                            , false);
//                } else if (cycleNumber == 2) {
//                    robot.runBlocking(new PathBuilder()
//                                    .addPath(new Path(
//                                            new BezierCurve(
//                                                    new Point(robotPose),
//                                                    robot.getFixedPoint(-20, robotPose.getY() - 15),
//                                                    new Point(shootingPosition)
//                                            )
//                                    ))
//                                    .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
//                                    .addParametricCallback(.3, () -> robot.powerOffIntake())
//                                    .setPathEndTValueConstraint(.95)
//                                    .setZeroPowerAccelerationMultiplier(6)
//                            , false);
//                } else if (cycleNumber == 3) {
//                    if (afterGate) {
//                        robot.runBlocking(new PathBuilder()
//                                        .addPath(new Path(
//                                                new BezierCurve(
//                                                        new Point(robotPose),
//                                                        robot.getFixedPoint(5, robotPose.getY() - 10),
//                                                        new Point(shootingPosition)
//                                                )
//                                        ))
//                                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
//                                        .addParametricCallback(.3, () -> robot.powerOffIntake())
//                                        .setPathEndTValueConstraint(.95)
//                                        .setZeroPowerAccelerationMultiplier(7)
//                                , false);
//                    } else {
//                        robot.runBlocking(new PathBuilder()
//                                        .addPath(new Path(
//                                                new BezierCurve(
//                                                        new Point(robotPose),
////                                                        robot.getFixedPoint(-20, robotPose.getY() - 5),
//                                                        new Point(shootingPosition)
//                                                )
//                                        ))
//                                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
//                                        .addParametricCallback(.3, () -> robot.powerOffIntake())
//                                        .setPathEndTValueConstraint(.95)
//                                        .setZeroPowerAccelerationMultiplier(6)
//                                , false);
//                    }
//                }
//            }
//        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
//            shootingPosition = robot.getFixedPose(-35, -115, 0);
//            robot.updateShooterParameters(shootingPosition);
//
//            robot.runBlocking(new PathBuilder()
//                            .addPath(new Path(
//                                    new BezierLine(
//                                            new Point(robotPose),
//                                            new Point(shootingPosition)
//                                    )
//                            ))
//                            .addVariableHeadingInterpolation(0, robotPose.getHeading(), robotPose.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
//                            .setPathEndTValueConstraint(.97)
//                            .setPathEndVelocityConstraint(100)
//                            .setZeroPowerAccelerationMultiplier(6)
//                    , false);
//        }
//
//        robot.setIntakePower(1);
//        robot.stopAndAim();
//        robot.setIntakePower(1);
//
//        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
//            if (cycleNumber == 0) {
//                robot.waitForShooter(1200);
//            } else {
//                robot.waitForShooter(150);
//            }
//        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
//            if (cycleNumber == 0) {
//                robot.waitForShooter(1600);
//            } else {
//                robot.waitForShooter(600);
//            }
//
//            robot.waitForHeadingCorrection(.75, 300, 1);
//        }
//
//        robot.setIntakePower(1);
//        robot.safeSleep(25);
//        robot.startScoringCycle();
//        robot.setIntakePower(1);
//        headingErrors.add(Double.toString(Math.toDegrees(robot.getFollower().headingError)));
//
//        robot.safeSleep(autoStartPos == AutoStartPos.CLOSE_ZONE ? 750 : 2200);
//
//        robot.stopScoringCycle();
//        robot.safeSleep(50);
//
//        FollowerConstants.useSecondaryHeadingPID = oldUseSecondaryHeading;
//        FollowerConstants.holdPointHeadingScaling = oldHoldPointScaling;
//        FollowerConstants.secondaryHeadingPIDFCoefficients = oldSecondaryHeading;
//        FollowerConstants.headingPIDFSwitch = oldHeadingPIDFSwitch;
//
//        robot.update();
//    }
//
//    private void park() {
//        Pose robotPose = robot.getPose();
//
//        robot.stopScoringCycle();
//        robot.runBlocking(new PathBuilder()
//                        .addPath(new Path(
//                                new BezierLine(
//                                        new Point(robotPose),
//                                        robot.getFixedPoint(-25, autoStartPos == AutoStartPos.FAR_ZONE ? -95 : -75)
//                                )
//                        ))
//                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
//                        .setPathEndTValueConstraint(.98)
//                        .setPathEndVelocityConstraint(1)
//                        .setZeroPowerAccelerationMultiplier(5)
//                , true);
//
//        robot.safeSleep(350);
//    }
//}
