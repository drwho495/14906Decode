package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorRangefinder {
    private RevColorSensorV3 internalSensor;

    public ColorRangefinder(HardwareMap hardwareMap, String name) {
        internalSensor = hardwareMap.get(RevColorSensorV3.class, name);

        ((LynxI2cDeviceSynch) internalSensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public NormalizedRGBA getDetectedColor() {
        return internalSensor.getNormalizedColors();
    }

    public double getDistance(DistanceUnit unit) {
        return internalSensor.getDistance(unit);
    }
}
