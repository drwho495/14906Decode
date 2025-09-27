package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bedroBathing.follower.Follower;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AutoManager {
    private final Follower follower;
    private final LinearOpMode opMode;
    private Runnable updateRunnable = null;
    private final Vector mirrorPlanePos = new Vector();
    private final Vector mirrorPlaneNormal = new Vector();

    public AutoManager(Follower newFollower, LinearOpMode newOpMode) {
        follower = newFollower;
        opMode = newOpMode;
    }

    private void internalRun(PathBuilder path, boolean correctAfterFinished) {
        // mirror the path

        PathChain oldBuiltPath = path.build();
        PathBuilder newPath = new PathBuilder();

        if (mirrorPlaneNormal.getXComponent() == 0 && mirrorPlaneNormal.getYComponent() == 0) {
            newPath = path;
        } else {
            for (int i = 0; i < oldBuiltPath.size(); i++) {
                Path currentPath = oldBuiltPath.getPath(i);
                ArrayList<Point> controlPoints = currentPath.getControlPoints();
                ArrayList<Point> mirroredPoints = new ArrayList<>();
                Path mirroredPath = null;

                for (int cI = 0; i < controlPoints.size(); i++) {
                    Point currentPoint = controlPoints.get(i);
                    Pose pointPose = new Pose(currentPoint.getX(), currentPoint.getY());

                    mirroredPoints.add(new Point(pointPose.getMirroredCopy(mirrorPlanePos, mirrorPlaneNormal)));
                }

                if (controlPoints.size() > 2) {
                    BezierCurve mirroredCurve = new BezierCurve(mirroredPoints);
                    mirroredPath = new Path(mirroredCurve);
                } else if (controlPoints.size() == 2) {
                    BezierLine mirroredLine = new BezierLine(mirroredPoints.get(0), mirroredPoints.get(1));
                    mirroredPath = new Path(mirroredLine);
                }

                if (mirroredPath != null) {
                    if (!mirroredPath.usingTangentialHeading()) {
                        mirroredPath.setTangentHeadingInterpolation();
                        mirroredPath.setReversed(mirroredPath.getReversed());
                    } else {
                        mirroredPath.setLinearHeadingInterpolation(
                                new Pose(0, 0, currentPath.getHeadingGoal(0)).getMirroredCopy(mirrorPlanePos, mirrorPlaneNormal).getHeading(),
                                new Pose(0, 0, currentPath.getHeadingGoal(1)).getMirroredCopy(mirrorPlanePos, mirrorPlaneNormal).getHeading(),
                                currentPath.getHeadingEndTValue()
                        );
                    }
//                mirroredPath.setReversed(currentPath.getReversed());
                    mirroredPath.setPathEndTValueConstraint(mirroredPath.getPathEndTValueConstraint());
                    mirroredPath.setPathEndTranslationalConstraint(mirroredPath.getPathEndTranslationalConstraint());
                    mirroredPath.setZeroPowerAccelerationMultiplier(mirroredPath.getZeroPowerAccelerationMultiplier());

                    newPath.addPath(mirroredPath);
                    opMode.telemetry.addData("path is not null, size: ", newPath.build().size());
                } else {
                    opMode.telemetry.addLine("path is null");
                }
            }
        }

        follower.followPath(newPath.build(), correctAfterFinished);
        follower.update();

        while (follower.isBusy() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            follower.update();
            opMode.telemetry.update();

            if (updateRunnable != null) updateRunnable.run();
        }
    }

    public void setMirrorPlane(Pose plane) {
        mirrorPlanePos.setOrthogonalComponents(plane.getX(), plane.getY());

        // convert to normal
        double cos = Math.cos(plane.getHeading());
        double sin = Math.sin(plane.getHeading());

        mirrorPlaneNormal.setOrthogonalComponents(cos, sin);
    }

    public Pose getMirroredPose() {
        return follower.getPose().getMirroredCopy(mirrorPlanePos, mirrorPlaneNormal);
    }

    public void setMirroredPose(Pose newPose) {
        follower.setPose(newPose.getMirroredCopy(mirrorPlanePos, mirrorPlaneNormal));
    }

    public void runBlocking(PathBuilder path) {
        internalRun(path, true);
    }

    public void runBlocking(PathBuilder path, boolean correctAfterFinished) {
        internalRun(path, correctAfterFinished);
    }

    public void setUpdateMethod(Runnable newRunnable) {
        updateRunnable = newRunnable;
    }

    public void safeSleep(double time) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.time(TimeUnit.MILLISECONDS) < time && opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (updateRunnable != null) updateRunnable.run();
            follower.update();
        }
    }
}
