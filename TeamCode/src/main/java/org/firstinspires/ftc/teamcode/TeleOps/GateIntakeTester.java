package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "Debug: Gate Intake Tester", group = "Debug")
public class GateIntakeTester extends LinearOpMode {
    private RobotManager robot;
    private Pose gatePose = new Pose();
    private Pose intakePose = new Pose();
    private ElapsedTime timer = new ElapsedTime();
    private boolean runningGateIntake = false;
    private boolean isPushingGate = false;
    private final double timePerPath = 2000;
    private final double waitAfterGate = 100;
    private final double waitAfterIntake = 100;
    private final int numberOfCycles = 2; // 0 indexed
    private int cycleI = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.initialise();
        robot.enableManualShooterControl();

        waitForStart();

        robot.powerOffShooter();
        robot.setTransferSpeed(1);
        robot.disableAutoTransferStop();
        robot.disableHoodCompensation();
        robot.disableVelocityCompensation();

        robot.setAllianceSide(AllianceSides.RED);

        while (opModeIsActive()) {
            Pose robotPose = robot.getPose();

            if (gamepad1.psWasPressed()) robot.setPose(Parameters.RED_CLOSE_START);
            if (gamepad1.aWasPressed()) gatePose = robotPose;
            if (gamepad1.bWasPressed()) intakePose = robotPose;
            if (gamepad1.yWasPressed()) runningGateIntake = true;

            telemetry.addLine("Press A to set gate pose to current robot position.");
            telemetry.addLine("Press B to set intake pose to current robot position.");
            telemetry.addLine("Press Y to intake from the gate.");

            telemetry.addData("Robot X: ", robotPose.getX());
            telemetry.addData("Robot Y: ", robotPose.getY());
            telemetry.addData("Robot Heading: ", Math.toDegrees(robotPose.getHeading()));
            telemetry.addLine();
            telemetry.addData("Gate X: ", gatePose.getX());
            telemetry.addData("Gate Y: ", gatePose.getY());
            telemetry.addData("Gate Heading: ", Math.toDegrees(gatePose.getHeading()));
            telemetry.addData("Intake X: ", intakePose.getX());
            telemetry.addData("Intake Y: ", intakePose.getY());
            telemetry.addData("Intake Heading: ", Math.toDegrees(intakePose.getHeading()));
            telemetry.addLine();
            telemetry.addData("Current Cycle Num: ", cycleI);

            if (runningGateIntake) {
                if (cycleI > numberOfCycles && !robot.getFollower().isBusy()) {
                    cycleI = 0;
                    runningGateIntake = false;
                    isPushingGate = false;
                } else {
                    robot.setIntakePower(1);

                    if (!robot.getFollower().isBusy()) {
                        if (!isPushingGate) {
                            robot.runPassthrough(new PathBuilder()
                                    .addPath(new Path(
                                                    new BezierLine(
                                                            new Point(robotPose),
                                                            new Point(gatePose)
                                                    )
                                            )
                                    )
                                    .setPathEndTValueConstraint(.95)
                                    .addLinearHeadingInterpolation(robotPose.getHeading(), gatePose.getHeading())
                            );
                        } else {
                            robot.runPassthrough(new PathBuilder()
                                    .addPath(new Path(
                                                    new BezierLine(
                                                            new Point(robotPose),
                                                            new Point(intakePose)
                                                    )
                                            )
                                    )
                                    .setPathEndTValueConstraint(.95)
                                    .addLinearHeadingInterpolation(robotPose.getHeading(), intakePose.getHeading())
                            );

                            cycleI++;
                        }

                        isPushingGate = !isPushingGate;
                        timer.reset();
                    } else {
                        if (timer.time(TimeUnit.MILLISECONDS) >= timePerPath) {
                            robot.breakFollowing();
                        }
                    }
                }
            } else {
                robot.setIntakePower(0);
            }

            telemetry.update();
            robot.update();
        }
    }
}
