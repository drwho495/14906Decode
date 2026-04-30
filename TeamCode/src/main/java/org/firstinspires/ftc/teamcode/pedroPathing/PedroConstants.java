package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.Parameters;

@Configurable
public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .headingPIDFCoefficients(
                    new PIDFCoefficients(
                            1,
                            0,
                            0.09,
                            0
                    )
            )
            .secondaryHeadingPIDFCoefficients(
                    new PIDFCoefficients(
                            1.75,
                            0,
                            0.08,
                            0
                    )
            )
            .useSecondaryHeadingPIDF(true)
            .predictiveBrakingCoefficients(
                    new PredictiveBrakingCoefficients(
                            0.14,
                            .194302517631,
                            .0014581433
                    )
            )
            .holdPointHeadingScaling(1)
            .stuckTValue(.6)
            .stuckTimeout(750)
            .mass(11.34);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motorRF")
            .rightRearMotorName("motorRR")
            .leftRearMotorName("motorLR")
            .leftFrontMotorName("motorLF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(Parameters.ROBOT == 0 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD) // the old chassis is messed up
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.04)
            .yVelocity(60.21)
            .useVoltageCompensation(true)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-34)
            .strafePodX(-43)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PinpointLocalizer pinpointLocalizer = null;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            0,
            1.6,
            .3
    );

    public static Follower follower = null;

    public static Follower getFollower(HardwareMap hardwareMap) {
        Pose startingPosition = new Pose();

        if (pinpointLocalizer == null) {
            pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerConstants);
        } else {
            pinpointLocalizer.update();
            startingPosition = pinpointLocalizer.getPose();
        }

        PedroConstants.follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(pinpointLocalizer)
                .build();

        PedroConstants.follower.setPose(startingPosition);

        return PedroConstants.follower;
    }
}