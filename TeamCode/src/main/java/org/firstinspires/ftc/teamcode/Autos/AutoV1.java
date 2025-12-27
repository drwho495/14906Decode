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
import org.firstinspires.ftc.teamcode.bedroBathing.util.PIDFController;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

enum AutoStartPos {
    CLOSE_ZONE,
    FAR_ZONE
}

enum AutoShootPosition {
    MIDDLE_LINE,
    FAR_ZONE
}

@Autonomous(name = "Auto V1", group = "0", preselectTeleOp = "0: Main Teleop")
public class AutoV1 extends LinearOpMode {
    private RobotManager robot;
    private AutoStartPos autoStartPos = AutoStartPos.CLOSE_ZONE;
    private boolean clearGate = true;
    private boolean grabFromHumanPlayer = true;
    private ElapsedTime timer = new ElapsedTime();
    private ArrayList<String> headingErrors = new ArrayList<>();

    // 0 is the line furthest from the goal
    private void intakeFromTape(double number) {
        robot.stopShootElement();
        robot.setIntakePower(1);
        robot.update();

        Pose robotPose = robot.getPose();
        Pose intakeStart = new Pose();
        double intakeEndX = 10;

        if (number == 0) {
            intakeStart.setX(-20);
            intakeStart.setY(-98);
        } else if (number == 1) {
            intakeStart.setX(-20);
            intakeStart.setY(-74);
        } else if (number == 2) {
            intakeStart.setX(-20);
            intakeStart.setY(-50);
            intakeEndX = robot.getAllianceSide() == AllianceSides.RED ? 6 : 4;
        }

        if (number != 2) {
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

        robot.safeSleep(150);
        robot.powerOffIntake();

        if (number == 1 && clearGate) {
            robotPose = robot.getPose();
            Pose mirroredRobotPose = robot.getFixedPose(robotPose);

            double pushHeading = robot.getFixedHeading(270);

            robot.setMaxFollowerPower(1);
            robot.addPathTimeout(1000);

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

        robot.setMaxFollowerPower(1);
    }

    private void intakeFromHumanPlayer() {
        Pose robotPose = robot.getPose();

        robot.stopShootElement();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        if (autoStartPos == AutoStartPos.FAR_ZONE) {
            double endY = -126;
            double midY = -100;

            if (robot.getAllianceSide() == AllianceSides.BLUE) {
                endY = -124;
                midY = -90;
            }

            robot.addPathTimeout(6000);
            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            robot.getFixedPoint(-30, midY),
                                            robot.getFixedPoint(17, endY)
                                    )
                            ))
                            .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(-15))
                            .setPathEndTValueConstraint(.92)
                            .setZeroPowerAccelerationMultiplier(5)
                    , true);
            robot.safeSleep(1000);

            robot.setMaxFollowerPower(.5);
            robot.runBlocking(new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    robot.getFixedPoint(17, endY),
                                    robot.getFixedPoint(-10, -123)
                            )
                    ))
                    .addConstantHeadingInterpolation(robot.getFixedHeading(-15))
                    .setPathEndTValueConstraint(.92)
                    .setZeroPowerAccelerationMultiplier(5), false);
            robot.setMaxFollowerPower(1);

            robot.safeSleep(1000);
            robot.powerOffIntake();
            robot.powerOnShooter();

            robot.safeSleep(100);
        } else {
            double wallY = 17.5; // 14.2
            double endHeading = Math.toRadians(280);

            robot.setMaxFollowerPower(1);

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
                shootingPosition = robot.getFixedPose(-25, -33, Math.toRadians(180));
                robot.setIntakePower(1);
                robot.runBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                new Point(robotPose),
                                                new Point(shootingPosition)
                                        )
                                ))
                                .addVariableHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading(), robot::getHeadingToGoal)
                                .setPathEndTValueConstraint(.95)
                                .setPathEndVelocityConstraint(1)
                                .setZeroPowerAccelerationMultiplier(6)
                        , true);
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
                                    .addParametricCallback(.75, () -> robot.setIntakePower(1))
//                                    .addTangentHeadingInterpolation(0, true, .85)
                                    .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoal, 1)
                                    .setPathEndTValueConstraint(.9)
//                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , true);
                } else if (cycleNumber == 2) {
                    robot.runBlocking(new PathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    new Point(robotPose),
                                                    robot.getFixedPoint(-20, robotPose.getY() - 15),
                                                    new Point(shootingPosition)
                                            )
                                    ))
                                    .addParametricCallback(.5, () -> robot.setIntakePower(1))
                                    .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoal, 1)
                                    .setPathEndTValueConstraint(.9)
//                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , true);
                } else if (cycleNumber == 3) {
                    robot.runBlocking(new PathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    new Point(robotPose),
                                                    robot.getFixedPoint(-20, robotPose.getY() - 5),
                                                    new Point(shootingPosition)
                                            )
                                    ))
                                    .addParametricCallback(.3, () -> {
                                        robot.setIntakePower(1);
                                    })
//                                    .addTangentHeadingInterpolation(0, true, .85)
                                    .addVariableHeadingInterpolation(0, Math.toRadians(270), shootingPosition.getHeading(), robot::getHeadingToGoal, 1)
                                    .setPathEndTValueConstraint(.9)
//                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , true);
                }
            }
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-35, -115, Math.toRadians(robot.getAllianceSide() == AllianceSides.RED ? 250 : 248));

            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            new Point(shootingPosition)
                                    )
                            ))
                            .addLinearHeadingInterpolation(robotPose.getHeading(), shootingPosition.getHeading())
                            .setPathEndTValueConstraint(.99)
                            .setPathEndVelocityConstraint(.5)
                            .setZeroPowerAccelerationMultiplier(8)
                    , true);

            robot.waitForHeadingCorrection(.5, 2000, .15);
        }

        if (autoStartPos == AutoStartPos.FAR_ZONE && cycleNumber == 0) {
            robot.safeSleep(1200);
        } else {
            if (cycleNumber == 0) {
                robot.waitForShooter(1200);
            } else {
                robot.waitForShooter(200);
            }
        }

        robot.setIntakePower(1);
        robot.safeSleep(25);
        robot.startShootElement();
        headingErrors.add(Double.toString(Math.toDegrees(robot.getFollower().headingError)));
//        robot.waitForHeadingCorrection(1, 500, 1);

        robot.safeSleep(autoStartPos == AutoStartPos.CLOSE_ZONE ? 850 : 2200);

        robot.stopShootElement();
        robot.safeSleep(50);

        FollowerConstants.useSecondaryHeadingPID = oldUseSecondaryHeading;
        FollowerConstants.holdPointHeadingScaling = oldHoldPointScaling;
        FollowerConstants.secondaryHeadingPIDFCoefficients = oldSecondaryHeading;
        FollowerConstants.headingPIDFSwitch = oldHeadingPIDFSwitch;

        robot.update();
    }

    private void park() {
        Pose robotPose = robot.getPose();

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
        robot.disableManualShooting();
        robot.initialise();
        robot.disableAutoTransferStop();
        robot.enableDebugPrinting();
        robot.disableVelocityCompensation();
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.resetIMU();

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
            telemetry.addLine("Press O to enable/disable clearing the gate and the third cycle.");
            if (clearGate && autoStartPos == AutoStartPos.CLOSE_ZONE)
                telemetry.addLine("Press ▲ to enable/disable grabbing artifacts from the human player.");
            telemetry.addLine("Press D-Pad Down to change the robot's starting position.");
            telemetry.addLine();
            telemetry.addData("Current Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            if (clearGate && autoStartPos == AutoStartPos.CLOSE_ZONE)
                telemetry.addData("Grab from Human Player: ", grabFromHumanPlayer);
            telemetry.addData("Start Location: ", autoStartPos);
            if (autoStartPos == AutoStartPos.CLOSE_ZONE)
                telemetry.addData("Clear Gate Enabled: ", clearGate);
            telemetry.update();

            robot.update();
        }

        Parameters.IMU_RECALIBRATED = true;
        waitForStart();

        robot.setTransferSpeed(1);
        robot.disableHoodCompensation();

        timer.reset();
        Parameters.IMU_RECALIBRATED = true;
        Parameters.LAST_ALLIANCE_SIDE = robot.getAllianceSide();

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            robot.setPose(robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START);
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            robot.setPose(robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START);
        }

        shootBalls(0);

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            intakeFromTape(grabFromHumanPlayer ? 0 : 1);
            shootBalls(grabFromHumanPlayer ? 1 : 2);
            intakeFromTape(grabFromHumanPlayer ? 1 : 0);
            shootBalls(grabFromHumanPlayer ? 2 : 1);
            intakeFromTape(2);
            if (clearGate) shootBalls(3);
            if (grabFromHumanPlayer && clearGate) {
                intakeFromHumanPlayer();
                shootBalls(3);
            } else {
                park();
            }
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            intakeFromHumanPlayer();
            shootBalls(1);

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
