package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

enum AutoStartPos {
    ON_WALL_FACING_TOWARDS_GOAL,
    ON_WALL_FACING_TOWARDS_WALL
}

@Autonomous(name = "Auto V1", group = "0", preselectTeleOp = "0: Main Teleop")
public class AutoV1 extends LinearOpMode {
    private RobotManager robot;
    private AutoStartPos autoStartPos = AutoStartPos.ON_WALL_FACING_TOWARDS_GOAL;
    private boolean clearGate = true;
    private boolean invertedGrabOrder = false;
    private boolean clearAfterLine2 = false;
    private boolean grabFromHumanPlayer = false;

    // 0 is the line furthest from the goal
    private void intakeFromTape(double number) {
        robot.tryStopShootElement();
        robot.trySetIntakePower(1);
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
            intakeEndX = 6;
        }

        if (number != 2) {
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            robot.getFixedPoint(intakeStart)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , number == 2);
        } else {
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            robot.getFixedPoint(-40, -35),
                                            robot.getFixedPoint(intakeStart)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(10)
                    , number == 2);
        }

        if (number == 2) robot.safeSleep(150);

        robot.addPathTimeout(4000);
        robot.trySetMaxPower(.5);
        robot.tryRunBlocking(new PathBuilder()
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
        robot.tryPowerOffIntake();

        if ((number == 1 && !clearAfterLine2 || number == 2 && clearAfterLine2) && clearGate) {
            robotPose = robot.getPose();
            Pose mirroredRobotPose = robot.getFixedPose(robotPose);

            double pushHeading = robot.getFixedHeading(15);
            if (clearAfterLine2) pushHeading = robot.getFixedHeading(-25);

            robot.trySetMaxPower(.85);
            robot.addPathTimeout(2250);
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            // for this, we need to mirror the robot pose, make the transformation, then mirror it back
                                            // if we don't, the robot will drive forward closer to the scoring wall instead of away from it
                                            robot.getFixedPoint(mirroredRobotPose.getX() - 20, mirroredRobotPose.getY() - 15),
                                            robot.getFixedPoint(8.5, clearAfterLine2 ? -58 :-60)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
                            .setPathEndTValueConstraint(.95)
                            .setZeroPowerAccelerationMultiplier(5)
                    , false);

            robot.safeSleep(200);
        }

        robot.trySetMaxPower(1);
    }

    private void intakeFromHumanPlayer() {
        Pose robotPose = robot.getPose();

        robot.tryStopShootElement();
        robot.trySetIntakePower(1);
        robot.trySetMaxPower(1);
        robot.update();

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        new Point(10, -95)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(318))
                        .setPathEndTValueConstraint(.92)
                        .setZeroPowerAccelerationMultiplier(5)
                , false);

        robot.safeSleep(75);
        robot.trySetMaxPower(.5);

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(10, -95),
                                        new Point(10, -117)
                                )
                        ))
                        .addTemporalCallback(2, () -> {
                            robot.tryBreakFollowing();
                        })
                        .setConstantHeadingInterpolation(robot.getFixedHeading(318))
                        .setPathEndTValueConstraint(.9)
                        .setZeroPowerAccelerationMultiplier(7)
                , false);

        robot.safeSleep(500);
        robot.tryPowerOffIntake();
        robot.trySetMaxPower(1);

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        robot.getFixedPoint(10, -117),
                                        robot.getFixedPoint(-16, -115)
                                )
                        ))
                        .setLinearHeadingInterpolation(robot.getFixedHeading(318), robot.getFixedHeading(0))
                        .setPathEndTValueConstraint(.92)
                        .setZeroPowerAccelerationMultiplier(7)
                , true);

        robot.safeSleep(100);
    }

    private void shootBalls(double cycleNumber) {
        Pose shootingPosition = robot.getFixedPose(-31, -48, Math.toRadians(230));
        Pose robotPose = robot.getPose();

        robot.tryPowerOffIntake();
        robot.tryPowerOnShooter();
        robot.trySetShooterVelocity(4400);
        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_FAR);
        robot.trySetMaxPower(1);

        if (cycleNumber == 0) {
            if(robot.getAllianceSide() == AllianceSides.RED) {
                robot.tryRunBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                new Point(robotPose),
                                                new Point(shootingPosition)
                                        )
                                ))
                                .setLinearHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading())
                                .setPathEndTValueConstraint(.95)
                                .setPathEndVelocityConstraint(1)
                                .setZeroPowerAccelerationMultiplier(6)
                        , true);
            } else if(robot.getAllianceSide() == AllianceSides.BLUE) {
                robot.tryRunBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierCurve(
                                                new Point(robotPose),
                                                robot.getFixedPoint(-10, -15),
                                                new Point(shootingPosition)
                                        )
                                ))
                                .setLinearHeadingInterpolation(robot.getFixedHeading(220), shootingPosition.getHeading())
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

            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            robot.getFixedPoint(-20, robotPose.getY() + yOffset),
                                            new Point(shootingPosition)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), shootingPosition.getHeading())
                            .setPathEndTValueConstraint(.95)
                            .setPathEndVelocityConstraint(1)
                            .setZeroPowerAccelerationMultiplier(8)
                    , true);
        }

        robot.safeSleep(500);

        robot.tryStartShootElement();

        robot.trySetIntakePower(1);
//        robot.safeSleep(350);

//        robot.trySetHoodServoPos(65);
        robot.safeSleep(1000);

        robot.tryStopShootElement();
        robot.tryStopShootElement();
        robot.safeSleep(50);

        robot.update();
    }

    private void park() {
        Pose robotPose = robot.getPose();

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        robot.getFixedPoint(-25, -75)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
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
        robot.initialiseHardware();
        robot.initialisePedroPathing();
        robot.tryResetIMU();

        while (opModeInInit()) {
            robot.tryPowerOffShooter();

            if (gamepad1.aWasPressed()) {
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.BLUE ? AllianceSides.RED : AllianceSides.BLUE);
            }

            if (gamepad1.bWasPressed()) {
                clearGate = !clearGate;
            }

            if (gamepad1.yWasPressed()) {
                grabFromHumanPlayer = !grabFromHumanPlayer;
            }

            telemetry.addLine("Press X to change the robot's alliance.");
            telemetry.addLine("Press O to enable/disable clearing the gate and the third cycle.");
            if (clearGate) telemetry.addLine("Press ▲ to enable/disable grabbing artifacts from the human player.");
            telemetry.addLine();
            telemetry.addData("Current Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            if (clearGate) telemetry.addData("Grab from Human Player: ", grabFromHumanPlayer);
            telemetry.addData("Clear Gate Enabled: ", clearGate);
            telemetry.update();

            robot.update();
        }

        waitForStart();
        Parameters.AUTO_PROGRAM_HAS_RUN = true;
        robot.setPose(robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START);

        shootBalls(0);
        intakeFromTape(invertedGrabOrder ? 2 : 0);
        shootBalls(1);
        intakeFromTape(1);
        shootBalls(2);
        intakeFromTape(invertedGrabOrder ? 0 : 2);

        if (clearGate) shootBalls(3);

        robot.tryPowerOffShooter();

        if (grabFromHumanPlayer && clearGate)
            intakeFromHumanPlayer();
        else
            park();

        Parameters.AUTO_PROGRAM_END_POSITION = robot.getPose();
    }
}
