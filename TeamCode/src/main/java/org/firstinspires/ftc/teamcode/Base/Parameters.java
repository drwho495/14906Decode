package org.firstinspires.ftc.teamcode.Base;

// all parameters are to be static fields, and most will need to be immutable

import com.acmerobotics.dashboard.config.Config;

//import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Base.Helpers.PedroUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.ShooterCurve;
import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.bedroBathing.util.CustomPIDFCoefficients;

@Config
@Configurable
public class Parameters {
    public static long MOTOR_ENCODER_TIMEOUT = 100;
    public static long SHOOTER_FINGER_TIMEOUT = 500;
    public static double TRANSFER_STOP_CURRENT = 4.5;
    public static double AUTO_SCORE_MAX_VEL = -1;
    public static double AUTO_SCORE_ERROR = 10;
    public static double AUTO_SCORE_WAIT_TIME = 550;
    public static double CENTRIPETAL_VELOCITY_COMPENSATION_MULTIPLIER = 0.09;
    public static double VELOCITY_CORRECTION_MULTIPLIER = 0.6;
    public static double SHOOTER_READY_TOLERANCE = 120;
    public static double SHOOTER_DEFAULT_RPM = 4500;
    public static double MIN_SHOOT_DISTANCE = 45;
    public static double FAR_ZONE_DISTANCE = 120;
    public static double FAR_ZONE_TRANSFER_SPEED = .35;
    public static double SHOOTER_VOLTAGE_TARGET = 11.7;

    public static AllianceSides LAST_ALLIANCE_SIDE = AllianceSides.RED;
    public static Pose RED_CLOSE_START = new Pose(0, 0, Math.toRadians(220));
    public static Pose RED_FAR_START = new Pose(-25, -130, Math.toRadians(90));
    public static Pose BLUE_CLOSE_START = new Pose(-92.7, 0, PedroUtils.getMirroredPose(RED_CLOSE_START).getHeading());
    public static Pose BLUE_FAR_START = PedroUtils.getMirroredPose(RED_FAR_START);
    //    public static Pose RED_SHOOTER_GOAL = new Pose(20, 0);
    public static Pose SHOOTER_GOAL_CLOSE_RED_AIM = new Pose(10, 0);
    public static Pose SHOOTER_GOAL_FAR_RED_AIM = new Pose(10  , 4);
    public static Pose SHOOTER_GOAL_CLOSE_BLUE_AIM = new Pose(-105, 10);
    public static Pose SHOOTER_GOAL_FAR_BLUE_AIM = new Pose(-95, 7.5);
    public static Pose SHOOTER_GOAL_CLOSE_RED = new Pose(18, 8);
    public static Pose SHOOTER_GOAL_FAR_RED = new Pose(10, 4);
    public static Pose SHOOTER_GOAL_CLOSE_BLUE = new Pose(-110, 10);
    public static Pose SHOOTER_GOAL_FAR_BLUE = new Pose(-102, 7.5);
    public static CustomPIDFCoefficients preciseTurnCoeffs = new CustomPIDFCoefficients(
            2,
            0,
            0.02,
            0);

    public static double FINGER_SERVO_OPEN = 45;
    public static double FINGER_SERVO_CLOSED = 100;

    public static double HOOD_SERVO_DOWN = 20;
    public static double HOOD_SERVO_UP = 300;
    public static double HOOD_SERVO_POSITION_OFFSET = 12;

    public static double DEFAULT_AIM_OFFSET_RED = 0;
    public static double DEFAULT_AIM_OFFSET_BLUE = -1;

    public static double TURRET_DEADZONE_ANGLE_FROM_ZERO = Math.toRadians(25);
    public static double TURRET_ANGLE_MULTIPLIER = 0.9;
    public static double TURRET_SERVO_LEFT_ZERO_OFFSET = Math.toRadians(10);
    public static double TURRET_SERVO_RIGHT_ZERO_OFFSET = Math.toRadians(6);
    public static double TURRET_BACKLASH = Math.toRadians(14);
    public static double TURRET_TEST_ANGLE = Math.toRadians(180);

    public static Pose TELEOP_AUTO_SCORE_POSE = new Pose(-36, -50);
    public static double TELEOP_AUTO_SCORE_HEADING = 15;
    public static Pose TELEOP_AUTO_GATE_POSE = new Pose(11, -76);
    public static double TELEOP_AUTO_GATE_HEADING = 35;

    public static double SHOOTER_FAR_ZONE_VELOCITY = 4700;
    public static double SHOOTER_FAR_ZONE_HOOD_ANGLE = 160;
    public static ShooterCurve CLOSE_ZONE_CURVE = new ShooterCurve();
    public static ShooterCurve FAR_ZONE_CURVE = new ShooterCurve();
}
