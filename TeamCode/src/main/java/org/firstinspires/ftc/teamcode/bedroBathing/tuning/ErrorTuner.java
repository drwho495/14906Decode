package org.firstinspires.ftc.teamcode.bedroBathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

/**
 * This is the ErrorTuner autonomous OpMode.
 *
 * @author Austin Bacher - 14906 Leviathan Robotics
 * @version 1.0, 11/21/2024
 */
@Config
@Autonomous (name = "Error Tuner", group = "Autonomous Pathing Tuning")
public class ErrorTuner extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;
    private boolean runNextPath = false;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards, true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if(gamepad1.a) runNextPath = true;
        if(runNextPath && follower.isBusy()) runNextPath = false;

        if (!follower.isBusy() && runNextPath) {
            if (forward) {
                forward = false;
                follower.followPath(backwards, true);
            } else {
                forward = true;
                follower.followPath(forwards, true);
            }
        }

        Path currentPath = follower.getCurrentPath();
        double xError = follower.getPose().getX() - currentPath.getLastControlPoint().getX();
        double yError = follower.getPose().getY() - currentPath.getLastControlPoint().getY();
        double headingError = follower.getPose().getHeading() - currentPath.getHeadingGoal(1);

        telemetryA.addData("run next path", runNextPath);
        telemetryA.addData("x error", xError);
        telemetryA.addData("y error", yError);
        telemetryA.addData("heading error (degrees)", Math.toDegrees(headingError));
        telemetryA.addData("heading error (radians)", headingError);
        telemetryA.update();
//        follower.telemetryDebug(telemetryA);
    }
}
