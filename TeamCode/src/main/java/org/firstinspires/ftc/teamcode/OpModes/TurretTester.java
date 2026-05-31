package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.TurretControlPolicy;

@TeleOp(name = "Debug: Turret Tester", group = "Debug")
public class TurretTester extends LinearOpMode {
    private RobotManager robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.initialise();
//        robot.enableTurretRelativeControl();
        robot.enableTurret();
        robot.setTurretControlPolicy(TurretControlPolicy.CONSTANT);

        waitForStart();

        while (opModeIsActive()) {
            robot.setConstantTurretHeadingGoal(Parameters.TURRET_TEST_ANGLE);

            if (gamepad1.aWasPressed()) {
                robot.enableTurretRelativeControl();
            } else if (gamepad1.aWasReleased()) {
                robot.disableTurretRelativeControl();
            }

            robot.update();
            telemetry.update();
        }
    }
}
