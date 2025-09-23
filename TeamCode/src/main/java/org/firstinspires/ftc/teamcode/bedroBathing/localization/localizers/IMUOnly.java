package org.firstinspires.ftc.teamcode.bedroBathing.localization.localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

public class IMUOnly extends Localizer {
    private IMU imu;
    private double totalHeading = 0;

    public IMUOnly(HardwareMap map) {
        imu = map.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
    }

    @Override
    public Pose getPose() {
        return new Pose(0, 0, totalHeading);
    }

    @Override
    public Pose getVelocity() {
        return new Pose();
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector();
    }

    @Override
    public void setStartPose(Pose setStart) {

    }

    @Override
    public void setPose(Pose setPose) {

    }

    @Override
    public void update() {
        totalHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
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
    public void resetIMU() throws InterruptedException {
        imu.resetYaw();
    }
}
