package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.concurrent.TimeUnit;

//@Disabled
@TeleOp(name = "Debug: Gate Intake Tester", group = "Debug")
public class GateIntakeTester extends LinearOpMode {
    private RobotManager robot;
    private double gateX = 12;
    private double gateY = -74.5;
    private double gateHeading = 36.5;
    private int waitTime = 1000;
    private boolean rtSlowdown = false;
    private boolean ltSlowdown = false;

    private void shootBalls() {
        Pose shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
        Pose robotPose = robot.getPose();

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

        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(robotPose),
                                        robot.getFixedPoint(5, robotPose.getY() - 10),
                                        new Point(shootingPosition)
                                )
                        ))
                        .addConstantHeadingInterpolation(0, robotPose.getHeading(), .4)
                        .addLinearHeadingInterpolation(.4, robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition), 1)
                        .addParametricCallback(.1, () -> robot.powerOffIntake())
                        .setPathEndTValueConstraint(.95)
                        .setZeroPowerAccelerationMultiplier(7)
                , false);

        robot.stopAndAim();

        robot.waitForShooter(1200);
        robot.setIntakePower(1);
        robot.safeSleep(25);
        robot.shootElements();

        robot.safeSleep(850);

        robot.cancelShootElements();
        robot.safeSleep(50);

        FollowerConstants.useSecondaryHeadingPID = oldUseSecondaryHeading;
        FollowerConstants.holdPointHeadingScaling = oldHoldPointScaling;
        FollowerConstants.secondaryHeadingPIDFCoefficients = oldSecondaryHeading;
        FollowerConstants.headingPIDFSwitch = oldHeadingPIDFSwitch;

        robot.update();
    }

    private void gateIntake() {
        Pose robotPose = robot.getPose();

        robot.setMaxFollowerPower(1);
        robot.setIntakePower(1);
        robot.runBlocking(
                new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        new Point(robotPose),
                                        new Point(2, -80, Point.CARTESIAN),
                                        new Point(gateX, gateY, Point.CARTESIAN)
                                )
                        ))
                        .addLinearHeadingInterpolation(0, robotPose.getHeading(), robot.getFixedHeading(gateHeading), .5)
                        .addLinearHeadingInterpolation(.5, robot.getFixedHeading(gateHeading), robot.getFixedHeading(gateHeading), 1)
                        .setPathEndTValueConstraint(.98)
                        .setZeroPowerAccelerationMultiplier(7)
        );

        robot.safeSleep(waitTime);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.initialise();
        robot.setAllianceSide(AllianceSides.RED);
        robot.disableAutoTransferStop();
        robot.disableHoodCompensation();
        robot.disableVelocityCompensation();
        robot.enableAutomaticTeleopShooting();
        robot.resetIMU();
        robot.recalibrateIMU();
        robot.resetIMU();
        robot.safeSleep(250);

        while (opModeInInit()) {
            robot.setPose(Parameters.RED_CLOSE_START);
            robot.update();
        }

        waitForStart();

        robot.setTransferSpeed(1);
        boolean needsToShoot = true;

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                gateHeading += 1;
            }

            if (gamepad1.dpadDownWasPressed()) {
                gateHeading -= 1;
            }

            if (gamepad1.rightBumperWasPressed()) {
                gateX += 1;
            }

            if (gamepad1.leftBumperWasPressed()) {
                gateX -= 1;
            }

            if (gamepad1.dpadRightWasPressed()) {
                gateY += 1;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                gateY -= 1;
            }

            if (gamepad1.right_trigger > .1 && !rtSlowdown) {
                waitTime += 100;
                rtSlowdown = true;
            }

            if (gamepad1.right_trigger <= .1)
                rtSlowdown = false;

            if (gamepad1.left_trigger > .1 && !ltSlowdown) {
                waitTime -= 100;
                ltSlowdown = true;
            }

            if (gamepad1.left_trigger <= .1)
                ltSlowdown = false;

            telemetry.addData("Gate X (Bumpers): ", gateX);
            telemetry.addData("Gate Y (DPad left/right): ", gateY);
            telemetry.addData("Gate Heading (DPad up/down): ", gateHeading);
            telemetry.addData("Wait time (Triggers): ", waitTime);
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                if (needsToShoot) {
                    needsToShoot = false;

                    shootBalls();
                    robot.safeSleep(1000);
                    gateIntake();
                    shootBalls();
                } else {
                    gateIntake();
                    shootBalls();
                }
            }
        }
    }
}
