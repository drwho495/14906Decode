package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;

@TeleOp(name = "Debug: Teleop Turn Tuner", group = "Debug")
public class TeleopTurnTuner extends LinearOpMode {
    private Follower follower;
    private double headingGoal = 0;

    public void setDrivePowers(double x, double y, double heading, boolean fieldCentric, boolean headingLock) {
        if (!follower.isTeleopDrive()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        if (headingLock) {
            Pose robotPose = follower.getPose();
            double robotHeading = MathFunctions.normalizeAngle(robotPose.getHeading());
            double direction = MathFunctions.getTurnDirection(robotHeading, headingGoal);


            headingGoal = MathFunctions.normalizeAngle(headingGoal);

            heading = follower.getVectorCalculator().getHeadingVector(
                    MathFunctions.getSmallestAngleDifference(robotHeading, headingGoal) * direction,
                    robotPose,
                    headingGoal
            ).getMagnitude() * direction * .9;
        }

        follower.setTeleOpDrive(x, y, heading, !fieldCentric);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = PedroConstants.getFollower(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            setDrivePowers(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true,
                    gamepad1.a
            );

            telemetry.addData("Heading: ", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();

            follower.update();
        }
    }
}
