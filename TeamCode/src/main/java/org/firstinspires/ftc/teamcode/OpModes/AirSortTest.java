package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.OpModeState;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShootingStyle;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Debug: Air Sort Tester", group = "Debug")
public class AirSortTest extends LinearOpMode {
    private RobotManager robot;
    private boolean isAirsorting = false;
    private int shotNum = 0;
    private final double timePerShot = 500;
    private final int maxNumShots = 3;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.disableHoodCompensation();
        robot.disableAutoTransferStop();
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.setTransferSpeed(.7);
        robot.enableAutoShooterControl();
        robot.disableWaitForVelocityToShoot();
        robot.setState(OpModeState.GENERAL_CYCLE);
        robot.setAllianceSide(AllianceSides.RED);

        robot.initialise();

        robot.stopScoringCycle();
        robot.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.yWasPressed())
                robot.toggleShooter();

            if (isAirsorting) {
                if (shotNum >= maxNumShots) {
                    isAirsorting = false;
                    shotNum = 0;

                    robot.setIntakePower(0);
                    robot.stopScoringCycle();
                } else {
                    if (timer.time(TimeUnit.MILLISECONDS) >= timePerShot) {
                        shotNum++;

                        robot.setShootingStyle(ShootingStyle.LARGE_ARC);

//                        robot.setIntakePower(.5);
//                        robot.stopScoringCycle();
//                        robot.setIntakePower(.5);
                        timer.reset();
                    }
//                    } else if (timer.time(TimeUnit.MILLISECONDS) >= (timePerShot - 250)) {
//                        robot.setIntakePower(1);
//                        robot.startScoringCycle();
//                    }
                }
            } else {
                robot.stopScoringCycle();

                if (robot.isShooterOn() && gamepad1.aWasPressed()) {
                    isAirsorting = true;
                    shotNum = 0;

                    robot.setIntakePower(.15);
                    robot.setShootingStyle(ShootingStyle.LARGE_ARC);
//                    robot.stopScoringCycle();
//                    robot.powerIntakeOff();
                    robot.startScoringCycle();
                    robot.setIntakePower(.15);
                    timer.reset();
                }
            }

            telemetry.addData("Is Sorting: ", isAirsorting);
            telemetry.addData("Shot Number: ", shotNum);
            telemetry.addData("Is Shooting: ", robot.scoringCycleActive());
            telemetry.addData("Distance to Goal: ", robot.getDistanceToGoal());
            telemetry.update();

            robot.update();
        }
    }
}
