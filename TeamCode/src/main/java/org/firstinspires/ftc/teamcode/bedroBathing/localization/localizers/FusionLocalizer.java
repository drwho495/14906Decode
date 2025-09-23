package org.firstinspires.ftc.teamcode.bedroBathing.localization.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bedroBathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

import java.util.HashMap;

public class FusionLocalizer extends Localizer {
    private HashMap<Integer, Localizer> localizers = new HashMap<>();
    private int currentLocalizer = 0;
    private Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private HardwareMap hwMap;

    private void initLocalizers() {
        localizers.put(0, new TwoWheelLocalizer(hwMap, startPose));
        localizers.put(1, new OTOSLocalizer(hwMap, startPose));
    }

    public FusionLocalizer(HardwareMap hwMap) {
        this.hwMap = hwMap;
        this.startPose = startPose;
        initLocalizers();
    }

    public FusionLocalizer(HardwareMap hwMap, Pose startPose) {
        this.hwMap = hwMap;
        this.startPose = startPose;
        initLocalizers();
    }

    @Override
    public Pose getPose() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getPose();
    }

    @Override
    public Pose getVelocity() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getVelocityVector();
    }

    public Localizer getCurrentLocalizer() {
        return localizers.get(currentLocalizer);
    }

    public void setCurrentLocalizer(int newLocalizerKey) {
        currentLocalizer = newLocalizerKey;
    }

    @Override
    public void setStartPose(Pose setStart) {
        for (int i = 0; i < localizers.values().size(); i++) {
            Localizer localizer = (Localizer) localizers.values().toArray()[i];

            localizer.setStartPose(setStart);
        }
    }

    @Override
    public void setPose(Pose setPose) {
        for (int i = 0; i < localizers.values().size(); i++) {
            Localizer localizer = (Localizer) localizers.values().toArray()[i];

            localizer.setPose(setPose);
        }
    }

    @Override
    public void update() {
        for (int i = 0; i < localizers.values().size(); i++) {
            Localizer localizer = (Localizer) localizers.values().toArray()[i];

            localizer.update();
        }
    }

    @Override
    public double getTotalHeading() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        Localizer localizer = localizers.get(currentLocalizer);

        return localizer.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        for (int i = 0; i < localizers.values().size(); i++) {
            Localizer localizer = (Localizer) localizers.values().toArray()[i];

//            localizer.resetIMU();
        }
    }
}