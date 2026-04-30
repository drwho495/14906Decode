package org.firstinspires.ftc.teamcode.Base;

// all parameters are to be static fields, and most will need to be immutable

import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Base.Helpers.PedroUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.ShooterCurve;
import org.firstinspires.ftc.teamcode.bedroBathing.util.CustomPIDFCoefficients;

@Config
@Configurable
public class Parameters {
    public static double SHOOTER_READY_TOLERANCE = 60;
    public static double SHOOTER_DEFAULT_RPM = 4500;
    public static double MIN_SHOOT_DISTANCE = 47;
    public static double FAR_ZONE_DISTANCE = 120;
    public static double FAR_ZONE_TRANSFER_SPEED = .6;
    public static double SHOOTER_VOLTAGE_TARGET = 11.7;
    public static boolean TELEOP_UPDATE_SHOOTER_PARAMS = false;

    public static AllianceSides LAST_ALLIANCE_SIDE = AllianceSides.RED;
    public static Pose RED_CLOSE_START = new Pose(0, 0, Math.toRadians(220));
    public static Pose RED_FAR_START = new Pose(-29.66, -120.83, Math.toRadians(-90));
    //    public static Pose RED_SHOOTER_GOAL = new Pose(20, 0);
    public static Pose SHOOTER_GOAL_CLOSE_RED = new Pose(18, 4);
    public static Pose SHOOTER_GOAL_FAR_RED = new Pose(10, 4);
    public static Pose SHOOTER_GOAL_CLOSE_BLUE = new Pose(-110.7, 4);
    public static Pose SHOOTER_GOAL_FAR_BLUE = new Pose(-109, 7.5);
    public static Pose BLUE_CLOSE_START = new Pose(-92.7, 0, PedroUtils.getMirroredPose(RED_CLOSE_START).getHeading());
    public static Pose BLUE_FAR_START = PedroUtils.getMirroredPose(RED_FAR_START);
    public static CustomPIDFCoefficients preciseTurnCoeffs = new CustomPIDFCoefficients(
            2,
            0,
            0.02,
            0);

    public static double FINGER_SERVO_OPEN = 45;
    public static double FINGER_SERVO_CLOSED = 100;

    // 0 is the robot with the unpocketed chassis, 1 is the robot with the pocketed chassis
    public static int ROBOT = 1;

    public static double HOOD_SERVO_DOWN = 20;
    public static double HOOD_SERVO_FAR = 50;
    public static double HOOD_SERVO_UP = 120;
    public static double HOOD_SERVO_DEFAULT = HOOD_SERVO_FAR;

    public static double DEFAULT_AIM_OFFSET_RED = 0;
    public static double DEFAULT_AIM_OFFSET_BLUE = -1;

    public static double SHOOTER_FAR_ZONE_VELOCITY = 4850;
    public static double SHOOTER_FAR_ZONE_HOOD_ANGLE = 55;
    public static ShooterCurve CLOSE_ZONE_CURVE = new ShooterCurve();
    public static ShooterCurve FAR_ZONE_CURVE = new ShooterCurve();
}
