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
    private boolean grabFromHumanPlayer = true;

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

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        new Point(intakeStart)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), 0)
                        .setPathEndTValueConstraint(.95)
                        .setZeroPowerAccelerationMultiplier(10)
                , number == 2);

        if (number == 2) robot.safeSleep(150);

        robot.trySetMaxPower(.5);
        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(intakeStart),
                                        new Point(intakeEndX, intakeStart.getY(), Point.CARTESIAN)
                                )
                        ))
                        .setPathEndTValueConstraint(.95)
                        .setZeroPowerAccelerationMultiplier(10)
                , true);

        robot.safeSleep(150);
        robot.tryPowerOffIntake();

        if (number == 1 && clearGate) {
            robotPose = robot.getPose();

            robot.trySetMaxPower(.8);
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            new Point(robotPose.getX() - 20, robotPose.getY() - 15, Point.CARTESIAN),
                                            new Point(8.5, -60)
                                    )
                            ))
                            .addTemporalCallback(4, () -> {
                                robot.tryBreakFollowing();
                            })
                            .setLinearHeadingInterpolation(robotPose.getHeading(), Math.toRadians(15))
                            .setPathEndTValueConstraint(.92)
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
                                        new Point(12, -95)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), Math.toRadians(318))
                        .setPathEndTValueConstraint(.92)
                        .setZeroPowerAccelerationMultiplier(5)
                , false);

        robot.safeSleep(75);
        robot.trySetMaxPower(.5);

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(12, -95),
                                        new Point(12, -117)
                                )
                        ))
                        .addTemporalCallback(2, () -> {
                            robot.tryBreakFollowing();
                        })
                        .setConstantHeadingInterpolation(Math.toRadians(318))
                        .setPathEndTValueConstraint(.9)
                        .setZeroPowerAccelerationMultiplier(7)
                , false);

        robot.safeSleep(500);
        robot.tryPowerOffIntake();
        robot.trySetMaxPower(1);

        robot.tryRunBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(12, -117),
                                        new Point(-16, -115)
                                )
                        ))
                        .setLinearHeadingInterpolation(Math.toRadians(318), 0)
                        .setPathEndTValueConstraint(.92)
                        .setZeroPowerAccelerationMultiplier(7)
                , true);

        robot.safeSleep(100);
    }

    private void shootBalls(double cycleNumber) {
        Pose shootingPosition = new Pose(-31, -48, Math.toRadians(230));
        Pose robotPose = robot.getPose();

        robot.tryPowerOffIntake();
        robot.tryPowerOnShooter();
        robot.trySetShooterVelocity(4300);
        robot.trySetHoodServoPos(45);
        robot.trySetMaxPower(1);

        if (cycleNumber == 0) {
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(robotPose),
                                            new Point(shootingPosition)
                                    )
                            ))
                            .setLinearHeadingInterpolation(Math.toRadians(220), shootingPosition.getHeading())
                            .setPathEndTValueConstraint(.95)
                            .setPathEndVelocityConstraint(1)
                            .setZeroPowerAccelerationMultiplier(6)
                    , true);
        } else if (cycleNumber >= 1) {
            robot.tryRunBlocking(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(robotPose),
                                            new Point(-20, robotPose.getY() - 15),
                                            new Point(shootingPosition)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), shootingPosition.getHeading())
                            .setPathEndTValueConstraint(.95)
                            .setPathEndVelocityConstraint(1)
                            .setZeroPowerAccelerationMultiplier(8)
                    , true);
        }

        robot.safeSleep(650);

        robot.tryStartShootElement();

        robot.trySetIntakePower(1);
        robot.safeSleep(350);

        robot.trySetHoodServoPos(65);
        robot.safeSleep(1200 - 350);

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
                                        new Point(-25, -75)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), 0)
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
        intakeFromTape(0);
        shootBalls(1);
        intakeFromTape(1);
        shootBalls(2);
        intakeFromTape(2);

        if (clearGate) shootBalls(3);

        robot.tryPowerOffShooter();

        if (grabFromHumanPlayer && clearGate)
            intakeFromHumanPlayer();
        else
            park();

        Parameters.AUTO_PROGRAM_END_POSITION = robot.getPose();
    }
}
