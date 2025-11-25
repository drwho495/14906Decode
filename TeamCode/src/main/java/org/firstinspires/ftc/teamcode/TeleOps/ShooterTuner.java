package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@TeleOp(name = "DEBUG: Shooter Tuner")
public class ShooterTuner extends LinearOpMode {
    private RobotManager robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.initialise();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.psWasPressed()) robot.setPose(Parameters.RED_CLOSE_START);
            if (gamepad1.aWasPressed()) robot.toggleShooter();

            if (gamepad1.leftBumperWasPressed()) {
                robot.startShootElement();
                robot.setIntakePower(1);
            } else if (gamepad1.leftBumperWasReleased()) {
                robot.stopShootElement();
                robot.setIntakePower(0);
            }

            if (gamepad1.dpadRightWasPressed()) {
                robot.setShooterVelocity(robot.getShooterTargetVelocity() + 15);
            } else if (gamepad1.dpadLeftWasPressed()) {
                robot.setShooterVelocity(robot.getShooterTargetVelocity() - 15);
            }

            if (gamepad1.dpadUpWasPressed()) {
                robot.setHoodServoPos(robot.getHoodAngle() + 5);
            } else if (gamepad1.dpadDownWasPressed()) {
                robot.setHoodServoPos(robot.getHoodAngle() - 5);
            }

            telemetry.addData("Shooter Velocity Target: ", robot.getShooterTargetVelocity());
            telemetry.addData("Hood Angle: ", robot.getHoodAngle());
            telemetry.update();
            robot.update();
        }
    }
}
