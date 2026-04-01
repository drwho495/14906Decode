package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.MecanumDrive;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShootingStyle;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;

import java.util.concurrent.TimeUnit;

// use 0 to get this file teleop program at the top of the list on the dhub
@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    // please do NOT store the state in a local variable in any opmodes, ESPECIALLY AUTO.
    // this is because we might not be able to control that variable when we set the state
    // (callbacks with pedro pathing)

    private boolean canDrive = true;
    private boolean showDebugInfo = false;
    private boolean autoStartShooterEnabled = true;
    private boolean autoStartShooter = false;
    private boolean autoStartShootingStarted = false;
    private boolean autoStartShootingStopTimed = false;
    private boolean driverNotifiedOf3 = false;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private final ElapsedTime timer = new ElapsedTime();

    private double shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotManager robot = new RobotManager(this);
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.initialise();
//        ElapsedTime resetIMUTimer = null;
//
//        if (!Parameters.IMU_RECALIBRATED) {
//            robot.resetIMU();
//            telemetry.addLine("Resetting IMU...");
//            telemetry.update();
//            resetIMUTimer = new ElapsedTime();
//
//            Parameters.IMU_RECALIBRATED = true;
//        }

        waitForStart();

        robot.setTransferSpeed(1);
        robot.enableAutoTransferStop();
        robot.disableHoodCompensation();
        robot.enableWaitForVelocityToShoot();
        robot.disableOnlyShootInZone();
        robot.enableVelocityCompensation();

//        if (resetIMUTimer != null) {
//            while (resetIMUTimer.time(TimeUnit.MILLISECONDS) < 3000 && opModeIsActive()) {
//                robot.update();
//            }
//        }

        robot.setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        robot.powerOffShooter();

        robot.disableAutoHeading();

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.setHoodServoPos(Parameters.HOOD_SERVO_DEFAULT);
        robot.setDriverOffset(robot.getAllianceSide() == AllianceSides.BLUE ? 180 : 0);

        while (opModeIsActive() && !isStopRequested()) {
            Pose robotPose = robot.getPose();

            if (canDrive) {
                robot.setDrivePowers(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }

            if (gamepad1.optionsWasPressed()) {
                robot.setDriverOffset(Math.toDegrees(robot.getPose().getHeading()));
            }

            if (gamepad1.shareWasPressed()) {
                robot.recalibrateIMU();
            }

            if (gamepad1.psWasPressed()) robot.setPose(robot.getAllianceSide() == AllianceSides.BLUE ? Parameters.BLUE_CLOSE_START : Parameters.RED_CLOSE_START);

            if (gamepad2.aWasPressed()) {
                showDebugInfo = !showDebugInfo;
            }

            switch (robot.getState()) {
                case IDLE:
                    break;
                case INTAKE_SCORE:
                    int goalOffsetAddMultiplier = robot.getAllianceSide() == AllianceSides.BLUE ? -1 : 1;

                    if (gamepad2.dpadDownWasPressed()) {
                        robot.resetGoalOffset();
                    } else if (gamepad2.dpadLeftWasPressed()) {
                        robot.addGoalOffset(-1 * goalOffsetAddMultiplier);
                    } else if (gamepad2.dpadRightWasPressed()) {
                        robot.addGoalOffset(1 * goalOffsetAddMultiplier);
                    }

                    if (gamepad2.rightBumperWasPressed()) {
                        robot.disableWaitForVelocityToShoot();
                    } else if (gamepad2.rightBumperWasReleased()) {
                        robot.enableWaitForVelocityToShoot();
                    }

                    if (gamepad1.rightBumperWasPressed()) {
                        robot.setConstantTeleopHeading(robot.getFixedHeading(36.5));
                        robot.enableAutoHeading();
                    } else if (gamepad1.rightBumperWasReleased()) {
                        robot.useGoalAimHeading();
                        robot.disableAutoHeading();
                    }

                    if (gamepad1.right_trigger > .1 || gamepad1.a) {
                        robot.setIntakePower(1);
                    } else if (gamepad2.right_trigger > .1 && !robot.isShooting()) {
                        robot.setIntakePower(gamepad2.right_trigger);
                    } else if (gamepad1.left_trigger > .1) {
                        robot.setIntakePower(-gamepad1.left_trigger);
                    } else {
                        robot.powerOffIntake();
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        robot.startScoringCycle();
                    } else if (gamepad1.leftBumperWasReleased()) {
                        robot.stopScoringCycle();
                    }

                    if (gamepad1.bWasPressed()) {
                        if (robot.getShootingStyle() != ShootingStyle.UNJAM) {
                            robot.setShootingStyle(ShootingStyle.UNJAM);
                        } else {
                            robot.setShootingStyle(ShootingStyle.LARGE_ARC);
                        }
                    }

//                    if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
//                        shooterVelocity += 100;
//                    }

//                    if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
//                        shooterVelocity -= 100;
//                    }

                    shooterVelocity = Range.clip(shooterVelocity, 0, 6000);

                    if (gamepad1.dpadUpWasPressed()) {
                        robot.toggleShooter();
                    }

                    if (autoStartShooterEnabled) {
                        if ((robot.isTransferStalled() || robot.getTransferDisableTime() >= 250 || gamepad1.left_bumper || gamepad1.right_bumper) && !autoStartShooter) {
                            autoStartShooter = true;
                            autoStartShootingStarted = false;
                            autoStartShootingStopTimed = false;

                            robot.powerOnShooter();
                        }

                        if (autoStartShooter && !autoStartShootingStarted && robot.isShooting()) {
                            autoStartShootingStarted = true;
                        }

                        if (autoStartShooter && autoStartShootingStarted && !robot.isShooting()) {
                            if (!autoStartShootingStopTimed) shooterTimer.reset();

                            autoStartShootingStopTimed = true;
                        } else {
                            autoStartShootingStopTimed = false;
                        }

                        if (autoStartShooter && autoStartShootingStopTimed && shooterTimer.time(TimeUnit.MILLISECONDS) > 1000) {
                            autoStartShooter = false;
                            autoStartShootingStarted = false;
                            autoStartShootingStopTimed = false;
                            robot.powerOffShooter();
                        }
                    }

                    if (gamepad1.xWasPressed()) {
                        robot.useGoalAimHeading();
                        robot.enableAutoHeading();
                    } else if (gamepad1.xWasReleased()) {
                        robot.disableAutoHeading();
                    }

                    if (gamepad2.leftBumperWasPressed()) {
                        robot.forceCancelShooting();
                    }

                    if (!driverNotifiedOf3) {
                        if (robot.getHeldBallCount() == 3){
                            gamepad1.rumble(500);
                            driverNotifiedOf3 = true;
                        }
                    } else {
                        if (robot.getHeldBallCount() != 3) {
                            driverNotifiedOf3 = false;
                        }
                    }

                    break;
                case PARK:
                    break;
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.RED ? AllianceSides.BLUE : AllianceSides.RED);
                robot.setDriverOffset(robot.getAllianceSide() == AllianceSides.BLUE ? 180 : 0);
            }

            Double[] shooterRPMs = robot.getCurrentShooterVelocities();

            telemetry.addData("Distance To Goal: ", robot.getDistanceToGoal());
            telemetry.addData("Robot Alliance: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue Side" : "Red Side");
            telemetry.addData("Heading Lock Goal Offset: ", robot.getGoalOffset());

            if (showDebugInfo) {
                telemetry.addData("Target RPM: ", robot.getShooterTargetVelocity());
                telemetry.addData("Actual RPM 1: ", shooterRPMs[0]);
                telemetry.addData("Actual RPM 2: ", shooterRPMs[1]);
                telemetry.addData("Hood Angle: ", robot.getHoodAngle());
                telemetry.addData("Is Shooting: ", robot.isShooting());
                telemetry.addData("Angular Velocity: ", robot.getFollower().getAngularVelocity());
                telemetry.addData("Disable Time: ", robot.getTransferDisableTime());
                telemetry.addData("Robot X: ", robotPose.getX());
                telemetry.addData("Robot Y: ", robotPose.getY());
                telemetry.addData("Robot Heading: ", Math.toDegrees(robotPose.getHeading()));
                telemetry.addData("Number of Artifacts in the Intake: ", robot.getHeldBallCount());
                telemetry.addData("Loop Time: ", timer.time(TimeUnit.MILLISECONDS));
            }

            timer.reset();

            robot.update();
            telemetry.update();
        }
    }
}
