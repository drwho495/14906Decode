package org.firstinspires.ftc.teamcode.Base;

// all parameters are to be static fields, and most will need to be immutable

import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Base.Helpers.PedroUtils;
import org.firstinspires.ftc.teamcode.bedroBathing.util.CustomPIDFCoefficients;

@Config
public class Parameters {
    public final static double INTAKE_SPEED = 1;
    public final static double SHOOTER_DEFAULT_RPM = 4500;
    public static final double MIN_SHOOT_DISTANCE = 47;
    public static final double FAR_ZONE_DISTANCE = 110;
    public static final double SLOW_TRANSFER = .8;

    public static AllianceSides LAST_ALLIANCE_SIDE = AllianceSides.RED;
    public final static Pose RED_CLOSE_START = new Pose(0, 0, Math.toRadians(220));
    public final static Pose RED_CLOSE_AUTO_START = new Pose(9, -11.5, Math.toRadians(220));
    public final static Pose BLUE_CLOSE_AUTO_START = PedroUtils.getMirroredPose(RED_CLOSE_AUTO_START);
    public final static Pose RED_FAR_START = new Pose(-30.5, -125, Math.toRadians(180));
//    public final static Pose RED_SHOOTER_GOAL = new Pose(20, 0);
    public final static Pose SHOOTER_GOAL_CLOSE = new Pose(20, 3);
    public final static Pose SHOOTER_GOAL_FAR = new Pose(15, 6.5);
    public final static Pose RED_TELEOP_SHOOTING_GOAL = new Pose(-30, -40);
    public final static Pose BLUE_CLOSE_START = new Pose(-90, 0, PedroUtils.getMirroredPose(RED_CLOSE_START).getHeading());
    public final static Pose BLUE_FAR_START = PedroUtils.getMirroredPose(RED_FAR_START);
    public final static Pose BLUE_TELEOP_SHOOTING_GOAL = PedroUtils.getMirroredPose(RED_TELEOP_SHOOTING_GOAL);
    public static CustomPIDFCoefficients preciseTurnCoeffs = new CustomPIDFCoefficients(
            2,
            0,
            0.02,
            0);

    public final static double FINGER_SERVO_OPEN = 45;
    public final static double FINGER_SERVO_CLOSED = 100;

    public final static int ROBOT = 1;

    public final static double HOOD_SERVO_DOWN = 20;
    public final static double HOOD_SERVO_FAR = 50;
    public final static double HOOD_SERVO_UP = 120;
    public final static double HOOD_SERVO_DEFAULT = HOOD_SERVO_FAR;

    public static boolean IMU_RECALIBRATED = false;
    public static Pose OPMODE_END_POSITION = new Pose();

    public final static double BALL_SENSOR_TOLERANCE = 130;
}
