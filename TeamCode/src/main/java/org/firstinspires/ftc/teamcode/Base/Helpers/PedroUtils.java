package org.firstinspires.ftc.teamcode.Base.Helpers;


import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.MathFunctions;

public class PedroUtils {
    public static Pose getMirroredPose(Pose pose) {
        return new Pose(-94 - pose.getX(), pose.getY(), MathFunctions.normalizeAngle(Math.PI - pose.getHeading()));
    }
}
