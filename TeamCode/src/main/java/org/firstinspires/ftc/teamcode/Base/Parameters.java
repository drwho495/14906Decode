package org.firstinspires.ftc.teamcode.Base;

// all parameters are to be static fields, and most will need to be immutable

import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

public class Parameters {
    public final static double INTAKE_SPEED = 1;
    public final static double WRIST_DOWN = 0;
    public final static double WRIST_UP = 100;
    public final static double WRIST_MID_POS = 60;
    public final static double CLAW_GRAB = 10;
    public final static double CLAW_RELEASE = 40;
    public final static double SHOOTER_ARM_DOWN = 10;
    public final static double SHOOTER_ARM_UP = 60;
    public final static double SHOOTER_DEFAULT_RPM = 2400;
    public final static double GATE_CLOSED_POSITION = 65;
    public final static double GATE_OPEN_POSITION = 10;
    public final static Vector MID_PLANE_POS = new Vector().setOrthogonalComponents(0, -35);
    public final static Vector MID_PLANE_NORMAL = new Vector().setOrthogonalComponents(0, 1);
    public final static Pose   RED_SHOOTER_GOAL = new Pose(7, -15);
    public final static Pose   BLUE_SHOOTER_GOAL = RED_SHOOTER_GOAL.getMirroredCopy(MID_PLANE_POS, MID_PLANE_NORMAL);

    public static boolean      AUTO_PROGRAM_RUN = false;
}
