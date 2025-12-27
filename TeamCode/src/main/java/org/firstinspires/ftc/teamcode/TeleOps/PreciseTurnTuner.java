package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

@TeleOp(name = "Debug: Precise Turn Tuner", group = "Debug")
public class PreciseTurnTuner extends LinearOpMode {
    private RobotManager robot;

    public void runPath() {
        Pose robotPose = robot.getPose();
        Pose scoringPose = robot.getFixedPose(-25, -33, Math.toRadians(180));

        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients = Parameters.preciseTurnCoeffs;
        FollowerConstants.holdPointHeadingScaling = 1;

        robot.runBlocking(new PathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        new Point(robotPose),
                                        new Point(scoringPose)
                                )
                        ))
                        .addVariableHeadingInterpolation(robot.getFixedHeading(220), scoringPose.getHeading(), robot::getHeadingToGoal)
                        .setPathEndTValueConstraint(.95)
                        .setPathEndVelocityConstraint(1)
                        .setZeroPowerAccelerationMultiplier(6)
                , true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);
        robot.initialise();
        robot.resetIMU();

        waitForStart();

        robot.setAllianceSide(AllianceSides.RED);
        robot.setPose(Parameters.RED_CLOSE_START);

        while (opModeIsActive()) {
            FollowerConstants.secondaryHeadingPIDFCoefficients = Parameters.preciseTurnCoeffs;

            telemetry.addData("Secondary Heading P: ", FollowerConstants.secondaryHeadingPIDFCoefficients.P);
            telemetry.addData("Secondary Heading I: ", FollowerConstants.secondaryHeadingPIDFCoefficients.I);
            telemetry.addData("Secondary Heading D: ", FollowerConstants.secondaryHeadingPIDFCoefficients.D);
            telemetry.addData("Secondary Heading F: ", FollowerConstants.secondaryHeadingPIDFCoefficients.F);
            telemetry.addData("Heading Error: ", Math.toDegrees(robot.getFollower().headingError));
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                Pose robotPose = robot.getPose();

                FollowerConstants.useSecondaryHeadingPID = false;

                robot.runBlocking(new PathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                new Point(robotPose),
                                                new Point(Parameters.RED_CLOSE_START)
                                        )
                                ))
                                .addLinearHeadingInterpolation(robotPose.getHeading(), Parameters.RED_CLOSE_START.getHeading())
                                .setPathEndTValueConstraint(.98)
                                .setPathEndVelocityConstraint(.5)
                                .setZeroPowerAccelerationMultiplier(4.5)
                        , true);
            } else if (gamepad1.yWasPressed()) {
                runPath();
            }

            robot.update();
        }
    }
}
