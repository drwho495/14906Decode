package org.firstinspires.ftc.teamcode.Base;

// all parameters are to be static fields, and most will need to be immutable

import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

public class Parameters {
    public final static double INTAKE_SPEED = 1;
    public final static double INTAKE_IDLE = .3;
    public final static double SHOOTER_ARM_DOWN = 10;
    public final static double SHOOTER_ARM_UP = 60;
    public final static double SHOOTER_DEFAULT_RPM = 4500;

    public static AllianceSides LAST_ALLIANCE_SIDE = AllianceSides.RED;
    public final static Pose RED_CLOSE_START = new Pose(0, 0, Math.toRadians(220));
    public final static Pose RED_SHOOTER_GOAL = new Pose(15, 0);
    public final static Pose BLUE_CLOSE_START = new Pose(-92, 0, RED_CLOSE_START.getMirroredCopy().getHeading());
    public final static Pose BLUE_SHOOTER_GOAL = RED_SHOOTER_GOAL.getMirroredCopy();

    public final static double FINGER_SERVO_OPEN = 45;
    public final static double FINGER_SERVO_CLOSED = 100;

    public final static double HOOD_SERVO_DOWN = 20;
    public final static double HOOD_SERVO_FAR = 50;
    public final static double HOOD_SERVO_UP = 120;
    public final static double HOOD_SERVO_DEFAULT = HOOD_SERVO_FAR;

    public static boolean AUTO_PROGRAM_HAS_RUN = false;
    public static Pose AUTO_PROGRAM_END_POSITION = new Pose();
}
