package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.PedroManager;
import org.firstinspires.ftc.teamcode.Base.SubsystemManager;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

// use 0 to get this file teleop program at the top of the list on the dhub
@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    // please do NOT store the state in a local variable in any opmodes, ESPECIALLY AUTO.
    // this is because we might not be able to control that variable when we set the state
    // (callbacks with pedro pathing)

    private Gamepad lastGamepad1 = new Gamepad();
    private Gamepad lastGamepad2 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private boolean canDrive = true;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private double shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;

    @Override
    public void runOpMode() throws InterruptedException {
        SubsystemManager robot = new SubsystemManager(this);
        Follower follower = new Follower(this.hardwareMap);
        robot.initialiseHardware();

        waitForStart();

        robot.tryPowerOnShooter();

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
        follower.setAutoHeadingState(false);

        if (!Parameters.AUTO_PROGRAM_RUN) {
            follower.setPose(new Pose(0, 0, Math.toRadians(45))); // the starting position on the wall of the goal
        }

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_DOWN);

        while (opModeIsActive() && !isStopRequested()) {
            Pose robotPose = follower.getPose();

            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);


            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (canDrive) {
                follower.setTeleOpMovementVectors(gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        false );
            }

            if (gamepad1.optionsWasPressed()) follower.resetIMU();

            switch (robot.getState()) {
                case IDLE:
                    break;
                case INTAKE_SCORE:
                    follower.setTeleopHeadingGoal(PedroManager.getHeadingToPoint(Parameters.RED_SHOOTER_GOAL, robotPose));

                    telemetry.addData("auto heading goal: ", Math.toDegrees(PedroManager.getHeadingToPoint(Parameters.RED_SHOOTER_GOAL, robotPose)));
                    telemetry.addData("current heading: ", Math.toDegrees(robotPose.getHeading()));
                    telemetry.addData("pose x: ", robotPose.getX());
                    telemetry.addData("pose y: ", robotPose.getY());

                    if (currentGamepad1.right_trigger > .1) {
                        robot.trySetIntakePower(currentGamepad1.right_trigger);
                    } else if (currentGamepad1.left_trigger > .1) {
                        robot.trySetIntakePower(-currentGamepad1.left_trigger);
                    } else {
                        robot.tryPowerOffIntake();
                    }

                    if (currentGamepad1.left_bumper) {
                        robot.tryStartShootElement();
                    } else {
                        robot.tryStopShootElement();
                    }

                    if (currentGamepad1.dpadRightWasPressed() || currentGamepad2.dpadRightWasPressed()) {
                        shooterVelocity += 100;
                        robot.trySetShooterVelocity(shooterVelocity);
                    }

                    if (currentGamepad1.dpadLeftWasPressed() || currentGamepad2.dpadLeftWasPressed()) {
                        shooterVelocity -= 100;
                        robot.trySetShooterVelocity(shooterVelocity);
                    }

                    if (currentGamepad2.rightBumperWasPressed()) {
                        robot.tryPowerOnShooter();
                    } else if (currentGamepad2.rightBumperWasReleased()) {
                        robot.tryPowerOffShooter();
                    }

                    if (currentGamepad1.aWasPressed()) {
                        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_DOWN);
                    } else if (currentGamepad1.bWasPressed()) {
                        robot.trySetHoodServoPos(Parameters.HOOD_SERVO_FAR);
                    }

                    if (currentGamepad1.right_stick_button && !lastGamepad1.right_stick_button) {
                        follower.setAutoHeadingState(!follower.getAutoHeadingState());
                    }
                    break;
                case PARK:
                    break;
            }

            telemetry.addData("Target RPM: ", shooterVelocity);
            telemetry.addData("Shooter 1 RPM: ", robot.getShooterVelocities()[0]);
            telemetry.addData("Shooter 2 RPM: ", robot.getShooterVelocities()[1]);
            telemetry.addData("state: ", robot.getState());

            robot.update();
            follower.update();
            telemetry.update();
        }
    }
}
