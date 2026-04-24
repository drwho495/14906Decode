//package org.firstinspires.ftc.teamcode.TeleOps;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Base.AllianceSides;
//import org.firstinspires.ftc.teamcode.Base.Parameters;
//import org.firstinspires.ftc.teamcode.Base.RobotManager;
//
//@TeleOp(name = "Debug: Precise Turn Tuner", group = "Debug")
//public class PreciseTurnTuner extends LinearOpMode {
//    private RobotManager robot;
//
//    public void runPath() {
//        Pose robotPose = robot.getPose();
//        Pose scoringPose = robot.getFixedPose(-25, -33, Math.toRadians(180));
//
//        robot.runBlocking(robot.pathBuilder()
//                        .addPath(new Path(
//                                new BezierLine(
//                                        robotPose,
//                                        scoringPose
//                                )
//                        ))
//                , true);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new RobotManager(this);
//        robot.initialise();
//        robot.resetIMU();
//
//        waitForStart();
//
//        robot.setAllianceSide(AllianceSides.RED);
//        robot.setPose(Parameters.RED_CLOSE_START);
//
//        while (opModeIsActive()) {
//            FollowerConstants.secondaryHeadingPIDFCoefficients = Parameters.preciseTurnCoeffs;
//
//            telemetry.addData("Secondary Heading P: ", FollowerConstants.secondaryHeadingPIDFCoefficients.P);
//            telemetry.addData("Secondary Heading I: ", FollowerConstants.secondaryHeadingPIDFCoefficients.I);
//            telemetry.addData("Secondary Heading D: ", FollowerConstants.secondaryHeadingPIDFCoefficients.D);
//            telemetry.addData("Secondary Heading F: ", FollowerConstants.secondaryHeadingPIDFCoefficients.F);
//            telemetry.addData("Heading Error: ", Math.toDegrees(robot.getFollower().headingError));
//            telemetry.update();
//
//            if (gamepad1.aWasPressed()) {
//                Pose robotPose = robot.getPose();
//
//                FollowerConstants.useSecondaryHeadingPID = false;
//
//                robot.runBlocking(new PathBuilder()
//                                .addPath(new Path(
//                                        new BezierLine(
//                                                new Point(robotPose),
//                                                new Point(Parameters.RED_CLOSE_START)
//                                        )
//                                ))
//                                .addLinearHeadingInterpolation(robotPose.getHeading(), Parameters.RED_CLOSE_START.getHeading())
//                                .setPathEndTValueConstraint(.98)
//                                .setPathEndVelocityConstraint(.5)
//                                .setZeroPowerAccelerationMultiplier(4.5)
//                        , true);
//            } else if (gamepad1.yWasPressed()) {
//                runPath();
//            }
//
//            robot.update();
//        }
//    }
//}
