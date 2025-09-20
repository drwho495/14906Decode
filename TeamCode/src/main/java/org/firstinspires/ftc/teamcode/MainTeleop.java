package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.RobotManager;

public class MainTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotManager robot = new RobotManager();
        robot.initialiseHardware();

        waitForStart();
    }
}
