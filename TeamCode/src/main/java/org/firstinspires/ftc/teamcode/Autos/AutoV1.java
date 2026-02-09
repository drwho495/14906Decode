package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShootingStyle;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.bedroBathing.util.CustomPIDFCoefficients;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

enum AutoStartPos {
    CLOSE_ZONE,
    FAR_ZONE
}

enum AutoType {
    TWELVE_ARTIFACT,
    FIFTEEN_ARTIFACT_HUMAN_PLAYER_GRAB,
    FIFTEEN_ARTIFACT_GATE_INTAKE,
    FIFTEEN_ARTIFACT_DOUBLE_GATE_INTAKE,
    EIGHTEEN_ARTIFACT
}

@Autonomous(name = "Auto V1", group = "0", preselectTeleOp = "0: Main Teleop")
public class AutoV1 extends LinearOpMode {
    private RobotManager robot;
    private AutoStartPos autoStartPos = AutoStartPos.CLOSE_ZONE;
    private Pose startPose;
    private boolean clearGate = true;
    private boolean grabFromLine0 = true;
    private boolean grabFromHumanPlayer = true;
    private int grabFromGateCycles = 0;
    private boolean farGrabFromLine = true;
    private AutoType autoType = AutoType.FIFTEEN_ARTIFACT_HUMAN_PLAYER_GRAB;

    private ElapsedTime timer = new ElapsedTime();
    private ArrayList<String> headingErrors = new ArrayList<>();

    // 0 is the line furthest from the goal
    private void intakeFromTape(double number) {
        robot.disableShooting();
        robot.setIntakePower(1);
        robot.update();

        Pose robotPose = robot.getPose();
        Pose intakeStart = new Pose();
        double intakeEndX = 8;

        if (number == 0) {
            intakeStart.setX(-20);
            intakeStart.setY(-98);
        } else if (number == 1) {
            intakeStart.setX(-20);
            intakeStart.setY(-74);
        } else if (number == 2) {
            intakeStart.setX(-20);
            intakeStart.setY(-50);
            intakeEndX = robot.getAllianceSide() == AllianceSides.RED ? 5 : 4;
        }

        if (number != 2 && !farGrabFromLine) {
            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            robot.getFixedPoint(intakeStart)
                                    )
                            ))
                            .addTangentHeadingInterpolation(0, false, .3)
                            .addLinearHeadingInterpolation(.3, robotPose.getHeading(), robot.getFixedHeading(0), 1)
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , number == 2);
        } else {
            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            robot.getFixedPoint(intakeStart)
                                    )
                            ))
                            .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , true);

            robot.safeSleep(150);
        }

        robot.addPathTimeout(4000);
        robot.setMaxFollowerPower(1);
        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        robot.getFixedPoint(intakeStart),
                                        robot.getFixedPoint(intakeEndX, intakeStart.getY())
                                )
                        ))
                        .setPathEndTValueConstraint(.95)
                        .setZeroPowerAccelerationMultiplier(10)
                , true);

        if (number == 1) {
            if (clearGate) {
                clearGate();
            }

            return;
        }

        robot.safeSleep(150);
        robot.powerOffIntake();
        robot.setMaxFollowerPower(1);
    }

    private void intakeFromGate() {
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);

        Pose robotPose = robot.getPose();

        robot.addPathTimeout(2000);
        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(robotPose),
                                        robot.getFixedPoint(-18, -75),
                                        robot.getFixedPoint(14, -67)
                                )
                        ))
                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(75))
                        .setPathEndTValueConstraint(.9)
                        .setPathEndVelocityConstraint(1000)
                        .setZeroPowerAccelerationMultiplier(6)
                , false);

        robotPose = robot.getPose();

        robot.addPathTimeout(2000);
        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(robotPose),
                                        robot.getFixedPoint(-4, -84),
                                        robot.getFixedPoint(16, -87)
                                )
                        ))
                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(55))
                        .setPathEndTValueConstraint(.9)
                        .setPathEndVelocityConstraint(1000)
                        .setZeroPowerAccelerationMultiplier(2)
                , false);

        robot.safeSleep(350);
        robotPose = robot.getPose();

        robot.addPathTimeout(2000);
        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        robot.getFixedPoint(16, -78)
                                )
                        ))
                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(55))
                        .setPathEndTValueConstraint(.9)
                        .setPathEndVelocityConstraint(1000)
                        .setZeroPowerAccelerationMultiplier(2)
                , false);

        robot.safeSleep(100);

        robot.powerOffIntake();
    }

    private void clearGate() {
        Pose robotPose = robot.getPose();
        Pose mirroredRobotPose = robot.getFixedPose(robotPose);

        double pushHeading = robot.getFixedHeading(260);

        robot.powerOffIntake();
        robot.setMaxFollowerPower(1);
        robot.addPathTimeout(900);

        boolean oldValue = FollowerConstants.useSecondaryHeadingPID;
        FollowerConstants.useSecondaryHeadingPID = false;

        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(robotPose),
                                        // for this, we need to mirror the robot pose, make the transformation, then mirror it back
                                        // if we don't, the robot will drive forward closer to the scoring wall instead of away from it
                                        robot.getFixedPoint(mirroredRobotPose.getX() - 5, mirroredRobotPose.getY() - 2),
                                        robot.getFixedPoint(14, -59)
                                )
                        ))
                        .addLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
                        .setPathEndTValueConstraint(.95)
                        .setZeroPowerAccelerationMultiplier(5)
                , false);

        FollowerConstants.useSecondaryHeadingPID = oldValue;

        robot.safeSleep(200);
    }

    private void intakeFromHumanPlayer() {
        Pose robotPose = robot.getPose();

        robot.disableShooting();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        double wallY = 14; // 14.2
        double endHeading = robot.getFixedHeading(280);

        robot.setMaxFollowerPower(.8);

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            robot.runBlocking(new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(robotPose),
                                    robot.getFixedPoint(-40, -80),
                                    robot.getFixedPoint(wallY, -94)
                            )
                    ))
                    .addTangentHeadingInterpolation(0, false, .5)
                    .addConstantHeadingInterpolation(.5, endHeading, 1)
                    .setPathEndTValueConstraint(.9)
                    .setPathEndVelocityConstraint(100)
                    .setPathEndTimeoutConstraint(0)
                    .setZeroPowerAccelerationMultiplier(15), true);
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            robot.runBlocking(new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(robotPose),
                                    robot.getFixedPoint(wallY, -94)
                            )
                    ))
                    .addLinearHeadingInterpolation(0, robotPose.getHeading(), endHeading, 1)
                    .setPathEndTValueConstraint(.9)
                    .setPathEndVelocityConstraint(100)
                    .setPathEndTimeoutConstraint(0)
                    .setZeroPowerAccelerationMultiplier(15), true);
        }

        robot.setMaxFollowerPower(.95);
        robot.addPathTimeout(1500);
        robot.runBlocking(new PathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                robot.getFixedPoint(wallY, -94),
                                robot.getFixedPoint(wallY, -118)
                        )
                ))
                .addLinearHeadingInterpolation(0, robot.getPose().getHeading(), endHeading, 1)
                .setPathEndTValueConstraint(.95)
                .setPathEndVelocityConstraint(100)
                .setZeroPowerAccelerationMultiplier(4.5), false);

        robot.safeSleep(750);
        robot.powerOffIntake();
    }

    private void shootBalls(double cycleNumber) {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        robot.powerOffIntake();
        robot.powerOnShooter();
        robot.setMaxFollowerPower(1);

        CustomPIDFCoefficients oldSecondaryHeading = FollowerConstants.secondaryHeadingPIDFCoefficients;
        double oldHoldPointScaling = FollowerConstants.holdPointHeadingScaling;
        boolean oldUseSecondaryHeading = FollowerConstants.useSecondaryHeadingPID;
        double oldHeadingPIDFSwitch = FollowerConstants.headingPIDFSwitch;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients = Parameters.preciseTurnCoeffs;
        FollowerConstants.holdPointHeadingScaling = 1;
        FollowerConstants.headingPIDFSwitch = Math.PI / 20;

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            if (cycleNumber == 0) {
                robot.enableShooting(); // make sure that servo opens!
                robot.update();

                shootingPosition = robot.getFixedPose(-25, -33, Math.toRadians(180));

                robot.addPathTimeout(2000);
                robot.runBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                new Point(startPose),
                                                new Point(shootingPosition)
                                        )
                                ))
                                .addVariableHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading(), robot::getHeadingToGoal)
                                .setPathEndTValueConstraint(.9)
                                .setPathEndVelocityConstraint(5)
                                .setZeroPowerAccelerationMultiplier(6)
                        , false);
            } else {
                shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));

                if (cycleNumber == 1) {
                    robot.runBlocking(new PathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    new Point(robotPose),
                                                    robot.getFixedPoint(-20, robotPose.getY() - 15),
                                                    new Point(shootingPosition)
                                            )
                                    ))
//                                    .addParametricCallback(.75, () -> robot.setIntakePower(1))
//                                    .addTangentHeadingInterpolation(0, true, .85)
                                    .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
                                    .setPathEndTValueConstraint(.95)
//                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , false);
                } else if (cycleNumber == 2) {
                    robot.runBlocking(new PathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    new Point(robotPose),
                                                    robot.getFixedPoint(-20, robotPose.getY() - 15),
                                                    new Point(shootingPosition)
                                            )
                                    ))
//                                    .addParametricCallback(.5, () -> robot.setIntakePower(1))
                                    .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
                                    .setPathEndTValueConstraint(.95)
//                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , false);
                } else if (cycleNumber == 3) {
                    if (grabFromGateCycles != 0) {
                        robot.runBlocking(new PathBuilder()
                                        .addPath(new Path(
                                                new BezierLine(
                                                        new Point(robotPose),
                                                        new Point(shootingPosition)
                                                )
                                        ))
                                        .addParametricCallback(.3, () -> {
//                                        robot.setIntakePower(1);
                                        })
//                                    .addTangentHeadingInterpolation(0, true, .85)
                                        .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
                                        .setPathEndTValueConstraint(.9)
//                                    .setPathEndVelocityConstraint(1)
                                        .setZeroPowerAccelerationMultiplier(6)
                                , false);
                    } else {
                        robot.runBlocking(new PathBuilder()
                                        .addPath(new Path(
                                                new BezierCurve(
                                                        new Point(robotPose),
                                                        robot.getFixedPoint(-20, robotPose.getY() - 5),
                                                        new Point(shootingPosition)
                                                )
                                        ))
                                        .addParametricCallback(.3, () -> {
//                                        robot.setIntakePower(1);
                                        })
//                                    .addTangentHeadingInterpolation(0, true, .85)
                                        .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
                                        .setPathEndTValueConstraint(.9)
//                                    .setPathEndVelocityConstraint(1)
                                        .setZeroPowerAccelerationMultiplier(6)
                                , false);
                    }
                }
            }
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-35, -115, 0);

            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            new Point(shootingPosition)
                                    )
                            ))
                            .addVariableHeadingInterpolation(0, robotPose.getHeading(), robotPose.getHeading(), robot::getHeadingToGoalWhileFollowingPath, 1)
                            .setPathEndTValueConstraint(.97)
                            .setPathEndVelocityConstraint(100)
                            .setZeroPowerAccelerationMultiplier(6)
                    , false);
        }

        robot.stopAndAim();

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            if (cycleNumber == 0) {
                robot.waitForShooter(1200);
            } else {
                robot.waitForShooter(200);
            }
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            if (cycleNumber == 0) {
                robot.waitForShooter(1600);
            } else {
                robot.waitForShooter(600);
            }

            robot.waitForHeadingCorrection(.75, 300, 1);
        }

        robot.setIntakePower(1);
        robot.safeSleep(25);
        robot.enableShooting();
        headingErrors.add(Double.toString(Math.toDegrees(robot.getFollower().headingError)));
//        robot.waitForHeadingCorrection(1, 500, 1);

        robot.safeSleep(autoStartPos == AutoStartPos.CLOSE_ZONE ? 850 : 2200);

        robot.disableShooting();
        robot.safeSleep(50);

        FollowerConstants.useSecondaryHeadingPID = oldUseSecondaryHeading;
        FollowerConstants.holdPointHeadingScaling = oldHoldPointScaling;
        FollowerConstants.secondaryHeadingPIDFCoefficients = oldSecondaryHeading;
        FollowerConstants.headingPIDFSwitch = oldHeadingPIDFSwitch;

        robot.update();
    }

    private void park() {
        Pose robotPose = robot.getPose();

        robot.disableShooting();
        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        robot.getFixedPoint(-25, autoStartPos == AutoStartPos.FAR_ZONE ? -95 : -75)
                                )
                        ))
                        .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                        .setPathEndTValueConstraint(.98)
                        .setPathEndVelocityConstraint(1)
                        .setZeroPowerAccelerationMultiplier(5)
                , true);

        robot.safeSleep(350);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.setTransferSpeed(1);
        robot.disableManualShooterControl();
        robot.disableAutoTransferStop();
        robot.disableDebugPrinting();
        robot.disableVelocityCompensation();
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.enableHoodCompensation();
        robot.disableOnlyShootInZone();
        robot.disableWaitForVelocityToShoot();
        robot.enablePoweredHold();

        robot.initialise();
        robot.recalibrateIMU();

        robot.enableShooting();
        robot.update();
        robot.update();
        robot.update();

        int autoTypeSelection = autoType.ordinal();

        while (opModeInInit()) {
            robot.powerOffShooter();

            if (gamepad1.aWasPressed())
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.BLUE ? AllianceSides.RED : AllianceSides.BLUE);

            if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
                if (gamepad1.bWasPressed())
                    clearGate = !clearGate;

                if (gamepad1.yWasPressed())
                    grabFromHumanPlayer = !grabFromHumanPlayer;
            }

            if (gamepad1.dpadDownWasPressed())
                autoStartPos = ((autoStartPos == AutoStartPos.CLOSE_ZONE) ? AutoStartPos.FAR_ZONE : AutoStartPos.CLOSE_ZONE);

            telemetry.addLine("Press X to change the robot's alliance.");
            telemetry.addLine("Press DPad Up/Down to change Autonomous Mode.");

            if (gamepad1.dpadUpWasPressed()) {
                autoTypeSelection++;
            } else if (gamepad1.dpadDownWasPressed()) {
                autoTypeSelection--;
            }

            if (autoTypeSelection >= AutoType.values().length) {
                autoTypeSelection = (AutoType.values().length - 1);
            } else if (autoTypeSelection < 0) {
                autoTypeSelection = 0;
            }

            autoType = AutoType.values()[autoTypeSelection];
            telemetry.addData("Alliance Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            telemetry.addData("Autonomous Mode: ", autoType);
            telemetry.addData("Robot Heading: ", robot.getPose().getHeading());

            if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
                startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START;
            } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
                startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START;
            }

            robot.setPose(startPose);

            robot.update();
            telemetry.update();
        }

        robot.enableDebugPrinting();

        switch (autoType) {
            case TWELVE_ARTIFACT:
                clearGate = true;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 0;
                grabFromLine0 = true;
                break;
            case FIFTEEN_ARTIFACT_HUMAN_PLAYER_GRAB:
                clearGate = true;
                grabFromHumanPlayer = true;
                grabFromGateCycles = 0;
                grabFromLine0 = true;
                break;
            case FIFTEEN_ARTIFACT_GATE_INTAKE:
                clearGate = false;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 1;
                grabFromLine0 = true;
                break;
            case FIFTEEN_ARTIFACT_DOUBLE_GATE_INTAKE:
                clearGate = false;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 2;
                grabFromLine0 = false;
                break;
        }

        Parameters.IMU_RECALIBRATED = true;
        waitForStart();

        robot.update();
        robot.getPose();

        timer.reset();
        Parameters.IMU_RECALIBRATED = true;
        Parameters.LAST_ALLIANCE_SIDE = robot.getAllianceSide();

        shootBalls(0);

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            if (grabFromHumanPlayer) {
                if (grabFromLine0) {
                    intakeFromTape(0);
                    shootBalls(1);
                }
                intakeFromTape(1);
                shootBalls(2);
                intakeFromTape(2);
                shootBalls(3);

                if (clearGate) {
                    intakeFromHumanPlayer();
                    shootBalls(3);
                }
            } else {
                intakeFromTape(1);
                shootBalls(2);

                for (int i = 0; i < grabFromGateCycles; i++) {
                    intakeFromGate();
                    shootBalls(3);
                }

                if (grabFromLine0) {
                    intakeFromTape(0);
                    shootBalls(1);
                }

                intakeFromTape(2);

                if (clearGate || grabFromGateCycles != 0) {
                    shootBalls(3);
                }
            }

            park();
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            int shootOffset = 0;

            if (farGrabFromLine) {
                shootOffset = 1;

                intakeFromTape(0);
                shootBalls(1);
            }

            intakeFromHumanPlayer();
            shootBalls(1 + shootOffset);

            robot.powerOffShooter();
            robot.powerOffIntake();

            while (opModeIsActive() && timer.time(TimeUnit.SECONDS) <= 27) {
                robot.update();
            }

            park();
        }

        robot.powerOffShooter();
        robot.powerOffIntake();
        robot.update();

        double time = ((double) timer.time(TimeUnit.MILLISECONDS)) / 1000;

        while (opModeIsActive()) {
            telemetry.addData("Time: ", time);
            telemetry.addData("Heading Errors: ", String.join(", ", headingErrors));
            telemetry.update();
        }

        Parameters.OPMODE_END_POSITION = robot.getPose();
    }
}
