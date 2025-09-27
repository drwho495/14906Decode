package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
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
        RobotManager robot = new RobotManager(this);
        Follower follower = new Follower(this.hardwareMap);
        robot.initialiseHardware();

        waitForStart();

        robot.tryPowerOnShooter();
        robot.tryPrimeShoot();

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
        follower.resetIMU();

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.tryClawGrab();

        while (opModeIsActive() && !isStopRequested()) {
            lastGamepad1.copy(currentGamepad1);
            lastGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            robot.trySetCustomVelocity(shooterVelocity);

            if (canDrive) {
                follower.setTeleOpMovementVectors(gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false);
            }

            if (gamepad1.options) follower.resetIMU();

            switch (robot.getState()) {
                case IDLE:
                    break;
                case INTAKE_SCORE:
                    if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        robot.tryClawToggle();
                    }

                    if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        if (!robot.isTransfering()) {
                            robot.tryTransfer();
                        } else {
                            robot.tryCancelTransfer();
                        }
                    }

                    if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up) {
                        robot.tryToggleShooter();
                    }

                    if (currentGamepad1.a && !lastGamepad1.a) {
                        robot.tryToggleGate();
                    }

                    if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
                        shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;
                    }

                    if (currentGamepad1.left_trigger > .1) {
                        robot.tryWristHold();
                    } else {
                        robot.tryWristDown();
                    }

                    if (currentGamepad1.right_trigger > .1) {
                        robot.tryShootElement();
                    } else {
                        robot.tryPrimeShoot();
                    }

                    if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left) {
                        shooterVelocity -= 100;
                    } else if (currentGamepad1.dpad_right && !lastGamepad1.dpad_right) {
                        shooterVelocity += 100;
                    }
                    break;
                case PARK:
                    break;
            }

            telemetry.addData("Target RPM: ", shooterVelocity);
            telemetry.addData("state: ", robot.getState());

            robot.update();
            follower.update();
            telemetry.update();
        }
    }
}
