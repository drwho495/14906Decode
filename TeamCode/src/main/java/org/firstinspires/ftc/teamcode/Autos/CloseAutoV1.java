package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Base.AutoManager;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

enum CloseAutoStartPos {
    ON_WALL_FACING_TOWARDS_GOAL,
    ON_WALL_FACING_TOWARDS_WALL
}

@Autonomous(name = "Close Zone Auto V1", group = "0", preselectTeleOp = "0: Main Teleop")
public class CloseAutoV1 extends LinearOpMode {
    private AutoManager auto;
    private Follower follower;
    private RobotManager robot;
    private CloseAutoStartPos autoStartPos = CloseAutoStartPos.ON_WALL_FACING_TOWARDS_GOAL;
    private boolean isBlue = true;
    private final Pose startPose = new Pose(0, -35, Math.toRadians(90));
    private Gamepad lastGamepad1 = new Gamepad();
    private Gamepad currentGamepad1 = new Gamepad();

    public void shootBalls(double cycleNumber) {
        if (cycleNumber == 1) {
            if (autoStartPos == CloseAutoStartPos.ON_WALL_FACING_TOWARDS_GOAL) {
                Pose followerPose = follower.getPose();

                auto.runBlocking(new PathBuilder()
                        .addPath(
                                new Path(
                                        new BezierLine(
                                                new Point(startPose),
                                                new Point(10, 8)
                                        )
                                )
                        )
                        .setPathEndTValueConstraint(.95)
                        .setLinearHeadingInterpolation(followerPose.getHeading(), Math.toRadians(-45))
                        .setZeroPowerAccelerationMultiplier(3), true
                );
            }
        } else {

        }

        auto.safeSleep(250);

        for (int i = 0; i <= 3; i++) {
            robot.tryShootElement(1);

            while (opModeIsActive() && robot.isShooting()) {
                robot.update();
                follower.update();
            }

            auto.safeSleep(300);
            robot.tryPrimeShoot();

            if (i == 0) robot.tryTransfer();

            auto.safeSleep(800);
        }
    }

    public void park() {
        Pose followerPose = follower.getPose();

        auto.runBlocking(new PathBuilder()
                .addPath(
                        new Path(
                                new BezierCurve(
                                        new Point(auto.getMirroredPose()),
                                        new Point(20, -10),
                                        new Point(80, -8)
                                )
                        )
                )
                .setPathEndTValueConstraint(.95)
                .setZeroPowerAccelerationMultiplier(5)
                .setLinearHeadingInterpolation(followerPose.getHeading(), 0)
//                .setTangentHeadingInterpolation()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(this.hardwareMap);
        follower.setMaxPower(1);

        auto = new AutoManager(follower, this);
        robot = new RobotManager(this);

        robot.initialiseHardware();

        auto.setUpdateMethod(() -> {

            robot.update();
//            telemetry.update();
        });


        robot.tryClawGrab();

        while (opModeInInit()) {
            robot.tryPowerOffShooter();

            lastGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !lastGamepad1.a) {
                isBlue = !isBlue;
            }

            telemetry.addData("Current Side: ", isBlue ? "Blue" : "Red");
            telemetry.update();

            robot.update();
        }

        waitForStart();

        if (isBlue) {
            auto.setMirrorPlane(startPose);
        }

        robot.tryCloseGate();

        if (autoStartPos == CloseAutoStartPos.ON_WALL_FACING_TOWARDS_GOAL) {
            auto.setMirroredPose(new Pose(-.36, 10, Math.toRadians(-45)));
        }

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.tryPowerOnShooter();
        robot.tryPrimeShoot();
        robot.trySetCustomVelocity(2400);

        shootBalls(1);
        telemetry.update();
        park();
    }
}
