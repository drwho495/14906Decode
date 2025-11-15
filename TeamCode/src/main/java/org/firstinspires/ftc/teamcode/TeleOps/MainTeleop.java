package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

// use 0 to get this file teleop program at the top of the list on the dhub
@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    // please do NOT store the state in a local variable in any opmodes, ESPECIALLY AUTO.
    // this is because we might not be able to control that variable when we set the state
    // (callbacks with pedro pathing)

    private boolean canDrive = true;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private double shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotManager robot = new RobotManager(this);
        robot.initialiseHardware();
        robot.initialisePedroPathing();

        waitForStart();

        robot.tryPowerOffShooter();

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.tryDisableAutoHeading();

        if (!Parameters.AUTO_PROGRAM_HAS_RUN) {
            robot.setPose(new Pose(0, 0, Math.toRadians(0)));
        } else {
            robot.setPose(Parameters.AUTO_PROGRAM_END_POSITION);
        }

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_DOWN);

        while (opModeIsActive() && !isStopRequested()) {
            Pose robotPose = robot.getPose();

            if (canDrive) {
                robot.trySetDrivePowers(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }

            if (gamepad1.optionsWasPressed()) {
                robotPose.setHeading(0);

                robot.setPose(robotPose);
            };
            if (gamepad1.psWasPressed()) robot.setPose(Parameters.RED_CLOSE_START);

            switch (robot.getState()) {
                case IDLE:
                    break;
                case INTAKE_SCORE:
                    robot.tryUseGoalAimHeading();

                    telemetry.addData("auto heading goal: ", Math.toDegrees(robot.getHeadingToGoal()));
                    telemetry.addData("current heading: ", Math.toDegrees(robotPose.getHeading()));
                    telemetry.addData("pose x: ", robotPose.getX());
                    telemetry.addData("pose y: ", robotPose.getY());

                    if (gamepad1.right_trigger > .1) {
                        robot.trySetIntakePower(gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > .1) {
                        robot.trySetIntakePower(-gamepad1.left_trigger);
                    } else {
                        robot.tryPowerOffIntake();
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        robot.tryStartShootElement();
                    } else if (gamepad1.leftBumperWasReleased()) {
                        robot.tryStopShootElement();
                    }

                    if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
                        shooterVelocity += 100;
                    }

                    if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
                        shooterVelocity -= 100;
                    }

                    shooterVelocity = Range.clip(shooterVelocity, 0, 6000);

                    if ((gamepad2.getGamepadId() != -1 && gamepad2.rightBumperWasPressed()) || (!robot.isShooterOn() && gamepad1.dpadUpWasPressed())) {
                        robot.tryPowerOnShooter();
                    } else if ((gamepad2.getGamepadId() != -1 && gamepad2.rightBumperWasReleased()) || (robot.isShooterOn() && gamepad1.dpadUpWasPressed())) {
                        robot.tryPowerOffShooter();
                    }

                    if (gamepad1.rightBumperWasPressed()) {
                        robot.tryEnableAutoHeading();
                    } else if (gamepad1.rightBumperWasReleased()) {
                        robot.tryDisableAutoHeading();
                    }

                    if (gamepad1.aWasPressed()) {
                        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_DOWN);
                    } else if (gamepad1.bWasPressed()) {
                        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_FAR);
                    }

                    robot.trySetShooterVelocity(shooterVelocity);
                    break;
                case PARK:
                    break;
            }

            telemetry.addData("Target RPM: ", shooterVelocity);
            telemetry.addData("Shooter 1 RPM: ", robot.getShooterVelocities()[0]);
            telemetry.addData("Shooter 2 RPM: ", robot.getShooterVelocities()[1]);
            telemetry.addData("state: ", robot.getState());

            robot.update();
            telemetry.update();
        }
    }
}
