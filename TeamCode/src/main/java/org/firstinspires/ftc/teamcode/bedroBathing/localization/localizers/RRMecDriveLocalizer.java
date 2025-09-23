package org.firstinspires.ftc.teamcode.bedroBathing.localization.localizers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Base.MecanumDrive;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

public class RRMecDriveLocalizer extends Localizer {
    private MecanumDrive mecanumDrive;
    private HardwareMap hwMap;

    public RRMecDriveLocalizer(HardwareMap hwMap) {
        this.hwMap = hwMap;
        mecanumDrive = new MecanumDrive(hwMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)), true);
    }

    public RRMecDriveLocalizer(HardwareMap hwMap, Pose newPose) {
        this.hwMap = hwMap;
        mecanumDrive = new MecanumDrive(hwMap, new Pose2d(new Vector2d(newPose.getX(), newPose.getY()), newPose.getHeading()), true);
    }

    @Override
    public Pose getPose() {
        return new Pose(mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, mecanumDrive.pose.heading.toDouble());
    }

    @Override
    public Pose getVelocity() {
        return null;
    }

    @Override
    public Vector getVelocityVector() {
        return null;
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        mecanumDrive.setPose(new Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading()));
    }

    @Override
    public void update() {
        mecanumDrive.updatePoseEstimate();
    }

    @Override
    public double getTotalHeading() {
        return mecanumDrive.pose.heading.toDouble();
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() {
        mecanumDrive.setPose(new Pose2d(mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toRadians(0)));
    }
}
