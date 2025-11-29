package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

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
    private boolean invertedGrabOrder = false;
    private boolean clearAfterLine2 = false;
    private boolean grabFromHumanPlayer = false;
    private ElapsedTime timer = new ElapsedTime();

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
                            .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , number == 2);
        } else {
            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            robot.getFixedPoint(-40, -55),
                                            robot.getFixedPoint(intakeStart)
                                    )
                            ))
                            .addLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , number == 2);
        }

        if (number == 2) robot.safeSleep(150);

        robot.addPathTimeout(4000);
        robot.setMaxFollowerPower(.5);
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

        if ((number == 1 && !clearAfterLine2 || number == 2 && clearAfterLine2) && clearGate) {
            robotPose = robot.getPose();
            Pose mirroredRobotPose = robot.getFixedPose(robotPose);

            double pushHeading = robot.getFixedHeading(15);
            if (clearAfterLine2) pushHeading = robot.getFixedHeading(-25);

            robot.setMaxFollowerPower(.85);
            robot.addPathTimeout(2250);
            robot.runBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            // for this, we need to mirror the robot pose, make the transformation, then mirror it back
                                            // if we don't, the robot will drive forward closer to the scoring wall instead of away from it
                                            robot.getFixedPoint(mirroredRobotPose.getX() - 20, mirroredRobotPose.getY() - 15),
                                            robot.getFixedPoint(8.5, clearAfterLine2 ? -58 :-60)
                                    )
                            ))
                            .addLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(5)
                    , false);

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
    }

    private void shootBalls(double cycleNumber) {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        robot.powerOffIntake();
        robot.powerOnShooter();
        robot.setMaxFollowerPower(1);

        FollowerConstants.secondaryHeadingPIDFCoefficients.P = 4;
        FollowerConstants.secondaryHeadingPIDFCoefficients.D = 0;
        FollowerConstants.headingPIDFSwitch = Math.PI / 10;
        FollowerConstants.holdPointHeadingScaling = 1;

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));

            if (cycleNumber == 0) {
                // jank >:(
                if (robot.getAllianceSide() == AllianceSides.RED) {
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
                } else if (robot.getAllianceSide() == AllianceSides.BLUE) {
                    robot.runBlocking(new PathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    new Point(robotPose),
                                                    robot.getFixedPoint(-5, -5),
                                                    new Point(shootingPosition)
                                            )
                                    ))
                                    .addVariableHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading(), robot::getHeadingToGoal)
                                    .setPathEndTValueConstraint(.95)
                                    .setPathEndVelocityConstraint(1)
                                    .setZeroPowerAccelerationMultiplier(6)
                            , true);
                }
            } else if (cycleNumber >= 1) {
                double yOffset = -15;

                if ((clearAfterLine2 && invertedGrabOrder && cycleNumber == 1)) {
                    yOffset = 25;
                }

                robot.runBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierCurve(
                                                new Point(robotPose),
                                                robot.getFixedPoint(-20, robotPose.getY() + yOffset),
                                                new Point(shootingPosition)
                                        )
                                ))
                                .addVariableHeadingInterpolation(robotPose.getHeading(), shootingPosition.getHeading(), robot::getHeadingToGoal)
                                .setPathEndTValueConstraint(.95)
                                .setPathEndVelocityConstraint(1)
                                .setZeroPowerAccelerationMultiplier(8)
                        , true);
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
                robot.safeSleep(800);
            } else {
                robot.safeSleep(500);
            }
        }

        FollowerConstants.holdPointHeadingScaling = .35;
        FollowerConstants.secondaryHeadingPIDFCoefficients.P = 1.5;
        FollowerConstants.secondaryHeadingPIDFCoefficients.D = 0;
        FollowerConstants.headingPIDFSwitch = Math.PI / 20;

        robot.startShootElement();

        robot.setIntakePower(1);
//        robot.safeSleep(350);

//        robot.setHoodServoPos(65);
        robot.safeSleep(autoStartPos == AutoStartPos.CLOSE_ZONE ? 1100 : 2200);

        robot.stopShootElement();
        robot.safeSleep(50);

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
            if (clearGate && autoStartPos == AutoStartPos.CLOSE_ZONE) telemetry.addLine("Press ▲ to enable/disable grabbing artifacts from the human player.");
            telemetry.addLine("Press D-Pad Down to change the robot's starting position.");
            telemetry.addLine();
            telemetry.addData("Current Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            if (clearGate && autoStartPos == AutoStartPos.CLOSE_ZONE) telemetry.addData("Grab from Human Player: ", grabFromHumanPlayer);
            telemetry.addData("Start Location: ", autoStartPos);
            if (autoStartPos == AutoStartPos.CLOSE_ZONE) telemetry.addData("Clear Gate Enabled: ", clearGate);
            telemetry.update();

            robot.update();
        }

        Parameters.AUTO_PROGRAM_HAS_RUN = true;
        waitForStart();
        timer.reset();
        Parameters.AUTO_PROGRAM_HAS_RUN = true;
        Parameters.LAST_ALLIANCE_SIDE = robot.getAllianceSide();

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            robot.setPose(robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START);
        } else if (autoStartPos == AutoStartPos.FAR_ZONE) {
            robot.setPose(robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START);
        }

        shootBalls(0);

        if (autoStartPos == AutoStartPos.CLOSE_ZONE) {
            intakeFromTape(invertedGrabOrder ? 2 : 0);
            shootBalls(1);
            intakeFromTape(1);
            shootBalls(2);
            intakeFromTape(invertedGrabOrder ? 0 : 2);
            if (clearGate) shootBalls(3);
            if (grabFromHumanPlayer && clearGate)
                intakeFromHumanPlayer();
            else {
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

        Parameters.OPMODE_END_POSITION = robot.getPose();
    }
}
