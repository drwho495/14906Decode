package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

import java.util.ArrayList;

@TeleOp(name = "Debug: Hardware Tester (SERVO ONLY)", group = "Debug")
public class HardwareTester extends LinearOpMode {
    private RobotManager robot;
    private int hardwareCursorIndex = 0;
    private ComplexServo selectedServo = null;
    private String selectedServoName = "";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.initialise();

        waitForStart();

        ArrayList<ComplexServo> robotServos = robot.getServos();
        final int robotServosSize = robotServos.size();

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) {
                hardwareCursorIndex++;
                selectedServo.disable();
            } else if (gamepad1.dpadLeftWasPressed()) {
                hardwareCursorIndex--;
                selectedServo.disable();
            }

            if (hardwareCursorIndex < 0) {
                hardwareCursorIndex = 0;
            } else if (hardwareCursorIndex >= robotServosSize) {
                hardwareCursorIndex = robotServosSize - 1;
            }

            selectedServo = robotServos.get(hardwareCursorIndex);

            if (selectedServo != null) {
                if (gamepad1.aWasPressed()) {
                    selectedServo.enable();
                    selectedServo.turnToAngle(0);
                }

                selectedServoName = selectedServo.getDeviceName();
            } else {
                selectedServoName = "N/A";
            }

            telemetry.addData("Selected Servo", selectedServoName);
            telemetry.update();
        }
    }
}
