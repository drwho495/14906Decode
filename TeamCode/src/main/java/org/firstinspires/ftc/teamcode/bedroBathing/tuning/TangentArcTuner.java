package org.firstinspires.ftc.teamcode.bedroBathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.DoubleHeadingTangentPath;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

@Config
@Autonomous (name = "Tangent Arc Tuner", group = "Autonomous Pathing Tuning")
public class TangentArcTuner extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE_X = 40;
    public static double DISTANCE_Y = 15;
    public static double TANGENT_LENGTH = 5;
    public static double DRIVE_POWER = 1;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new DoubleHeadingTangentPath(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE_X,DISTANCE_Y, Point.CARTESIAN)));
        forwards.setTangentHeadingInterpolation(0, 0, TANGENT_LENGTH);
        backwards = new Path(new DoubleHeadingTangentPath(new Point(DISTANCE_X,DISTANCE_Y, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setTangentHeadingInterpolation(0, 0, TANGENT_LENGTH);
        backwards.setReversed(true);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE_X
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
        follower.setMaxPower(DRIVE_POWER);
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        telemetryA.addData("heading (deg): ", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("heading goal at 1 (deg): ", Math.toDegrees(follower.getCurrentPath().getHeadingGoal(1)));
        telemetryA.addData("heading goal at current (deg): ", Math.toDegrees(follower.getCurrentPath().getHeadingGoal(follower.getCurrentTValue())));
        telemetryA.addData("t: ", follower.getCurrentTValue());
        telemetryA.addData("tangent: ", follower.getCurrentPath().getEndTangent());
        telemetryA.update();
//        follower.telemetryDebug(telemetryA);
    }
}
