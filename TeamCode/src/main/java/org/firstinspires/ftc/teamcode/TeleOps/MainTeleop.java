package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

import java.util.concurrent.TimeUnit;

// use 0 to get this file teleop program at the top of the list on the dhub
@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    // please do NOT store the state in a local variable in any opmodes, ESPECIALLY AUTO.
    // this is because we might not be able to control that variable when we set the state
    // (callbacks with pedro pathing)

    private boolean canDrive = true;
    private boolean autoStartShooterEnabled = true;
    private boolean autoStartShooter = false;
    private boolean autoStartShootingStarted = false;
    private boolean autoStartShootingStopTimed = false;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private final ElapsedTime timer = new ElapsedTime();

    private double shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotManager robot = new RobotManager(this);
        robot.initialise();

        if (!Parameters.AUTO_PROGRAM_HAS_RUN) {
            robot.resetIMU();
            telemetry.addLine("Resetting IMU...");
            telemetry.update();
        }

        waitForStart();

        robot.setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        robot.powerOffShooter();

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.disableAutoHeading();

        if (!Parameters.AUTO_PROGRAM_HAS_RUN) {
//            robot.setPose(new Pose(0, 0, Math.toRadians(0)));
        } else {
//            robot.setPose(Parameters.OPMODE_END_POSITION);
        }

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.setHoodServoPos(Parameters.HOOD_SERVO_DEFAULT);

        while (opModeIsActive() && !isStopRequested()) {
            Pose robotPose = robot.getPose();

            if (canDrive) {
                robot.setDriverOffset(robot.getAllianceSide() == AllianceSides.BLUE ? 180 : 0);

                robot.setDrivePowers(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }

            if (gamepad1.optionsWasPressed()) {
                robotPose.setHeading(0);

                robot.setPose(robotPose);
            };
            if (gamepad1.psWasPressed()) robot.setPose(robot.getAllianceSide() == AllianceSides.BLUE ? Parameters.BLUE_CLOSE_START : Parameters.RED_CLOSE_START);

            switch (robot.getState()) {
                case IDLE:
                    break;
                case INTAKE_SCORE:
                    robot.useGoalAimHeading();

                    telemetry.addData("auto heading goal: ", Math.toDegrees(robot.getHeadingToGoal()));
                    telemetry.addData("current heading: ", Math.toDegrees(robotPose.getHeading()));
                    telemetry.addData("pose x: ", robotPose.getX());
                    telemetry.addData("pose y: ", robotPose.getY());

                    if (gamepad1.right_trigger > .1) {
                        robot.setIntakePower(gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > .1) {
                        robot.setIntakePower(-gamepad1.left_trigger);
                    } else {
                        robot.powerOffIntake();
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        robot.startShootElement();
                    } else if (gamepad1.leftBumperWasReleased()) {
                        robot.stopShootElement();
                    }

                    if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
                        shooterVelocity += 100;
                    }

                    if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
                        shooterVelocity -= 100;
                    }

                    shooterVelocity = Range.clip(shooterVelocity, 0, 6000);

                    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                        robot.toggleShooter();
                    }

                    if (gamepad2.rightBumperWasPressed()) {
                        robot.powerOnShooter();
                    } else if (gamepad2.leftBumperWasPressed()) {
                        robot.powerOffShooter();
                    }

//                        if (gamepad1.aWasPressed()) {
//                            robot.powerOnShooter();
//                        } else if (gamepad1.aWasReleased()) {
//                            robot.powerOffShooter();
//                        }

                    if (autoStartShooterEnabled) {
                        if ((robot.isTransferStalled() || robot.getTransferDisableTime() >= 250) && !autoStartShooter) {
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

                    if (gamepad1.rightBumperWasPressed()) {
                        robot.enableAutoHeading();
                    } else if (gamepad1.rightBumperWasReleased()) {
                        robot.disableAutoHeading();
                    }

                    if (gamepad1.aWasPressed()) {
                        robot.setHoodServoPos(Parameters.HOOD_SERVO_DOWN);
                    } else if (gamepad1.bWasPressed()) {
                        robot.setHoodServoPos(Parameters.HOOD_SERVO_FAR);
                    }

                    robot.setShooterVelocity(shooterVelocity);
                    break;
                case PARK:
                    break;
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.RED ? AllianceSides.BLUE : AllianceSides.RED);
            }

            Double[] shooterVelocities = robot.getCurrentShooterVelocities();

            telemetry.addData("Robot Alliance: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue Side" : "Red Side");
            telemetry.addData("Distance To Goal: ", robot.getDistanceToGoal());
            telemetry.addData("Target RPM: ", robot.getShooterTargetVelocity());
            telemetry.addData("Disable Time: ", robot.getTransferDisableTime());
            telemetry.addData("Shooter 1 RPM: ", shooterVelocities[0]);
            telemetry.addData("Shooter 2 RPM: ", shooterVelocities[1]);
            telemetry.addData("Loop Time: ", timer.time(TimeUnit.MILLISECONDS));

            timer.reset();

            robot.update();
            telemetry.update();
        }
    }
}
