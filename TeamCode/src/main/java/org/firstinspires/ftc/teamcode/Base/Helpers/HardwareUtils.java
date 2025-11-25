package org.firstinspires.ftc.teamcode.Base.Helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;

import java.util.HashMap;
import java.util.Map;

public class HardwareUtils {
    public static final Map<Object, Double> previousValues = new HashMap<>();

    /**
     * This method (Provided by DrPixelCat24 on Discord) will run optimize the call to the writeFunction method.
     *
     */
    public static void optimizeMethod(double position, Object device, Consumer<Double> writeFunction) {
        double prevValue = previousValues.getOrDefault(device, Double.NaN);
        if (Double.isNaN(prevValue) || position != prevValue) {
            writeFunction.accept(position);
            previousValues.put(device, position);
        }
    }
}
