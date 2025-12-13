package org.firstinspires.ftc.teamcode.bedroBathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;

@Autonomous(name = "Multiple Heading Tuner Test", group = "Tuning")
public class MultipleHeadingTest extends LinearOpMode {
    private boolean forward = true;
    private double distanceX = 50;
    private double distanceY = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap);
        follower.resetIMU();

        waitForStart();
        follower.setPose(new Pose());

        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (forward) {
                    follower.followPath(new PathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            new Point(0, 0, Point.CARTESIAN),
                                            new Point(distanceX/2, -10, Point.CARTESIAN),
                                            new Point(distanceX, distanceY, Point.CARTESIAN)
                                    )
                            ))
                            .addTangentHeadingInterpolation(0,false, .5)
                            .addLinearHeadingInterpolation(.5, 0, Math.toRadians(-90), 1)
                            .build());

                    forward = false;
                } else {
                    follower.followPath(new PathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            new Point(distanceX, distanceY, Point.CARTESIAN),
                                            new Point(0, 0, Point.CARTESIAN)
                                    )
                            ))
                            .addLinearHeadingInterpolation(0, Math.toRadians(-90), 0, .5)
                            .addConstantHeadingInterpolation(.5, 0, 1)
                            .build());

                    forward = true;
                }
            }

            follower.update();
        }
    }
}
