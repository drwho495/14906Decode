package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Misc.TurretControlPolicy;

@TeleOp(name = "Debug: Turret Tester", group = "Debug")
public class TurretTester extends LinearOpMode {
    private RobotManager robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.initialise();
        robot.enableTurret();

        waitForStart();

        while (opModeIsActive()) {
            robot.setConstantTurretHeadingGoal(Parameters.TURRET_TEST_ANGLE, AngleUnit.RADIANS);

            if (gamepad1.aWasPressed()) {
                robot.disableTurretRelativeControl();
            } else if (gamepad1.aWasReleased()) {
                robot.enableTurretRelativeControl();
            }

            robot.update();
            telemetry.update();
        }
    }
}
