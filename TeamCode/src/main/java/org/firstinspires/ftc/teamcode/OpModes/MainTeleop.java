package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Misc.HeadingLockControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.OpModeState;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Misc.ShootingStyle;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterAimPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.TurretControlPolicy;

import java.util.concurrent.TimeUnit;

// use 0 to get this file teleop program at the top of the list on the dhub
@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    // please do NOT store the state in a local variable in any opmodes, ESPECIALLY AUTO.
    // this is because we might not be able to control that variable when we set the state
    // (callbacks with pedro pathing)

    private boolean canDrive = true;
    private boolean gateIntakeStateIsAutoScoring = false;
    private boolean gateIntakeStateIsGateIntaking = false;
    private boolean gateIntakeStateCanShootArtifacts = false;
    private Pose gateIntakeLocalScorePose = Parameters.TELEOP_AUTO_SCORE_POSE;
    private double gateIntakeLocalScoreHeading = Parameters.TELEOP_AUTO_SCORE_HEADING;
    private Pose gateIntakeLocalGatePose = Parameters.TELEOP_AUTO_GATE_POSE;
    private double gateIntakeLocalGateHeading = Parameters.TELEOP_AUTO_GATE_HEADING;
    private boolean showDebugInfo = true;
    private boolean autoStartShooterEnabled = true;
    private boolean autoStartShooter = false;
    private boolean autoStartShootingStarted = false;
    private boolean autoStartShootingStopTimed = false;
    private boolean manualTurretActive = false;
    private boolean manualTurretPositionLocked = false;
    private boolean driverNotifiedOf3 = false;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private final ElapsedTime timer = new ElapsedTime();

    private double shooterVelocity = Parameters.SHOOTER_DEFAULT_RPM;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotManager robot = new RobotManager(this);
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.initialise();
        robot.breakFollowing();

        waitForStart();

        robot.setTransferSpeed(1);
        robot.enableAutoTransferStop();
        robot.disableHoodCompensation();
        robot.enableWaitForVelocityToShoot();
        robot.disableOnlyShootInZone();
        robot.enableVelocityCompensation();
        robot.disablePoweredHold();
        robot.setAllianceSide(Parameters.LAST_ALLIANCE_SIDE);
        robot.powerShooterOff();
        robot.disableHeadingLock();
        robot.setState(OpModeState.GENERAL_CYCLE);
        robot.setTurretControlPolicy(TurretControlPolicy.AIM_AT_GOAL);
        robot.setShooterAimPolicy(ShooterAimPolicy.TURRET);
        robot.setDriverOffset(robot.getAllianceSide() == AllianceSides.BLUE ? 180 : 0);

        if (robot.getShooterAimPolicy() == ShooterAimPolicy.TURRET) {
            robot.enableTurret();
            robot.startAimingAtGoal();
        }

        while (opModeIsActive() && !isStopRequested()) {
            Pose robotPose = robot.getPose();

            if (canDrive) {
                robot.setDrivePowers(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            }

            if (gamepad1.optionsWasPressed()) {
                robot.setDriverOffset(Math.toDegrees(robot.getPose().getHeading()));
            }

            if (gamepad1.shareWasPressed() || gamepad2.shareWasPressed()) {
                robot.recalibrateIMU();
            }

            if (gamepad1.psWasPressed() || gamepad2.psWasPressed())
                robot.setPose(robot.getAllianceSide() == AllianceSides.BLUE ? Parameters.BLUE_CLOSE_START : Parameters.RED_CLOSE_START);

            if (gamepad2.aWasPressed()) {
                showDebugInfo = !showDebugInfo;
            }

            if (gamepad1.dpadRightWasPressed()) {
                robot.setState(OpModeState.SPECIALIZED_GATE_CYCLE);
            } else if (gamepad1.dpadLeftWasPressed()) {
                robot.setState(OpModeState.GENERAL_CYCLE);
            }

            if (gamepad2.xWasPressed()) {
                if (manualTurretActive) {
                    robot.startAimingAtGoal();

                    manualTurretActive = false;
                } else {
                    robot.stopAimingAtGoal();

                    manualTurretActive = true;
                }
            }

            if (manualTurretActive) {
                if (gamepad2.yWasPressed()) {
                    manualTurretPositionLocked = !manualTurretPositionLocked;
                }

                if (!manualTurretPositionLocked) {
                    robot.setConstantTurretHeadingGoal(
                            robot.getFixedHeading(
                                    Math.atan2(gamepad2.right_stick_y, gamepad2.right_stick_x) - ((3 * Math.PI) / 2),
                                    AngleUnit.RADIANS
                            ),
                            AngleUnit.RADIANS
                    );
                }
            }

            switch (robot.getState()) {
                case GENERAL_CYCLE:
                    int goalOffsetAddMultiplier = robot.getAllianceSide() == AllianceSides.BLUE ? -1 : 1;

                    if (robot.isStateStart()) {
                        robot.disableHeadingLock();

                        if (robot.getShooterControlPolicy() != ShooterControlPolicy.ROAMING) {
                            robot.setShooterControlPolicy(ShooterControlPolicy.ROAMING);
                        }
                    }

                    if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                        Parameters.CLOSE_ZONE_CURVE.build();
                        Parameters.FAR_ZONE_CURVE.build();
                    }

                    if (gamepad2.dpadDownWasPressed()) {
                        robot.resetGoalOffset();
                    } else if (gamepad2.dpadLeftWasPressed()) {
                        robot.addGoalOffset(-1 * goalOffsetAddMultiplier);
                    } else if (gamepad2.dpadRightWasPressed()) {
                        robot.addGoalOffset(1 * goalOffsetAddMultiplier);
                    }

                    if (gamepad2.rightBumperWasPressed()) {
                        robot.disableWaitForVelocityToShoot();
                    } else if (gamepad2.rightBumperWasReleased()) {
                        robot.enableWaitForVelocityToShoot();
                    }

                    if (gamepad1.rightBumperWasPressed()) {
                        robot.setHeadingLockControlPolicy(HeadingLockControlPolicy.CONSTANT);
                        robot.setConstantHeadingLockGoal(robot.getFixedHeading(36.5));

                        robot.enableHeadingLock();
                    } else if (gamepad1.rightBumperWasReleased()) {
                        robot.disableHeadingLock();
                    }

                    if (gamepad1.right_trigger > .1 || gamepad1.a) {
                        robot.setIntakePower(1);
                    } else if (gamepad2.right_trigger > .1 && !robot.scoringCycleActive()) {
                        robot.setIntakePower(gamepad2.right_trigger);
                    } else if (gamepad1.left_trigger > .1) {
                        robot.setIntakePower(-gamepad1.left_trigger);
                    } else {
                        robot.powerIntakeOff();
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        robot.startScoringCycle();
                    } else if (gamepad1.leftBumperWasReleased()) {
                        robot.stopScoringCycle();
                    }

                    shooterVelocity = Range.clip(shooterVelocity, 0, 6000);

                    if (gamepad1.dpadUpWasPressed()) {
                        robot.toggleShooter();
                    }

                    if (autoStartShooterEnabled) {
                        if ((robot.isTransferStopped() || robot.transferOverCurrentTime() > 300 || gamepad1.left_bumper || gamepad1.right_bumper) && !autoStartShooter) {
                            autoStartShooter = true;
                            autoStartShootingStarted = false;
                            autoStartShootingStopTimed = false;

                            robot.powerShooterOn();
                        }

                        if (autoStartShooter && !autoStartShootingStarted && robot.scoringCycleActive()) {
                            autoStartShootingStarted = true;
                        }

                        if (autoStartShooter && autoStartShootingStarted && !robot.scoringCycleActive()) {
                            if (!autoStartShootingStopTimed) shooterTimer.reset();

                            autoStartShootingStopTimed = true;
                        } else {
                            autoStartShootingStopTimed = false;
                        }

                        if (autoStartShooter && autoStartShootingStopTimed && shooterTimer.time(TimeUnit.MILLISECONDS) > 1000) {
                            autoStartShooter = false;
                            autoStartShootingStarted = false;
                            autoStartShootingStopTimed = false;
                            robot.powerShooterOff();
                        }
                    }

                    if (robot.getShooterAimPolicy() == ShooterAimPolicy.DRIVETRAIN) {
                        if (gamepad1.xWasPressed()) {
                            robot.startAimingAtGoal();
                        } else if (gamepad1.xWasReleased()) {
                            robot.stopAimingAtGoal();
                        }
                    }

                    if (gamepad2.leftBumperWasPressed()) {
                        robot.forceCancelShooting();
                    }
                    break;
                case SPECIALIZED_GATE_CYCLE:
                    boolean enableManualGatePosition = false;

                    if (robot.isStateStart()) {
                        robot.powerShooterOn();
                        robot.stopScoringCycle();
                        robot.setShooterControlPolicy(ShooterControlPolicy.MANUAL);
                        robot.setTransferSpeed(1);

                        gateIntakeStateIsAutoScoring = false;
                        gateIntakeStateIsGateIntaking = false;

                        robot.updateShooterParameters(robot.getFixedPose(Parameters.TELEOP_AUTO_SCORE_POSE));

                        enableManualGatePosition = true;
                    }

                    if (gamepad1.a && !gateIntakeStateIsAutoScoring) {
                        canDrive = false;
                        gateIntakeStateCanShootArtifacts = false;
                        gateIntakeStateIsAutoScoring = true;
                        gateIntakeStateIsGateIntaking = false;

                        robot.setMaxFollowerPower(1);
                        robot.breakFollowing();
                        robot.stopScoringCycle();
                        robot.runPassthrough(
                                robot.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        robotPose,
                                                        robot.getFixedPose(-10, -55),
                                                        robot.getFixedPose(gateIntakeLocalScorePose)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateIntakeLocalScoreHeading), .5)
                                        .setTValueConstraint(1)
                                        .setVelocityConstraint(1000)
                                        .addTemporalCallback(.1, () -> {
                                            robot.setIntakePower(0);
                                            robot.startScoringCycle(true);
                                        })
                        );
                    }

                    if (gamepad1.x && !gateIntakeStateIsGateIntaking) {
                        canDrive = false;
                        gateIntakeStateCanShootArtifacts = false;
                        gateIntakeStateIsAutoScoring = false;
                        gateIntakeStateIsGateIntaking = true;

                        robot.setMaxFollowerPower(1);
                        robot.breakFollowing();
                        robot.powerIntakeOn();
                        robot.stopScoringCycle();
                        robot.runPassthrough(
                                robot.pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        robotPose,
                                                        robot.getFixedPose(-15, -70),
                                                        robot.getFixedPose(gateIntakeLocalGatePose)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateIntakeLocalGateHeading), .2)
                                        .setTValueConstraint(1)
                                        .setVelocityConstraint(1000)
                        );
                    }

                    if (gateIntakeStateIsAutoScoring && robot.getFollower().getDistanceRemaining() < 5 && !gateIntakeStateCanShootArtifacts) {
                        robot.setIntakePower(1);

                        gateIntakeStateCanShootArtifacts = true;
                    }

                    if (!canDrive && (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1)) {
                        enableManualGatePosition = true;
                        canDrive = true;
                        gateIntakeStateIsAutoScoring = false;
                        gateIntakeStateIsGateIntaking = false;
                        gateIntakeStateCanShootArtifacts = false;

                        robot.breakFollowing();
                        robot.stopScoringCycle();
                    }

                    if (enableManualGatePosition) {
                        robot.powerIntakeOn();
                        robot.setConstantHeadingLockGoal(robot.getFixedHeading(35));
                        robot.enableHeadingLock();
                    }

                    break;
                case PARK:
                    break;
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.RED ? AllianceSides.BLUE : AllianceSides.RED);
                robot.setDriverOffset(robot.getAllianceSide() == AllianceSides.BLUE ? 180 : 0);
            }

            Double[] shooterRPMs = robot.getCurrentShooterVelocities();

            telemetry.addData("Distance To Goal: ", robot.getDistanceToGoal());
            telemetry.addData("Robot Alliance: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue Side" : "Red Side");
            telemetry.addData("Heading Lock Goal Offset: ", robot.getGoalOffset());
            telemetry.addData("Loop Time: ", timer.time(TimeUnit.MILLISECONDS));
            telemetry.addData("Manual Turret Enabled: ", manualTurretActive);

            if (showDebugInfo) {
                telemetry.addLine("! DEBUG !");
                telemetry.addData("Target RPM: ", robot.getShooterTargetVelocity());
                telemetry.addData("Actual RPM 1: ", shooterRPMs[0]);
                telemetry.addData("Actual RPM 2: ", shooterRPMs[1]);
                telemetry.addData("Hood Angle: ", robot.getHoodAngle());
                telemetry.addData("Is Shooting: ", robot.scoringCycleActive());
                telemetry.addData("Angular Velocity: ", robot.getFollower().getAngularVelocity());
                telemetry.addData("Robot X: ", robotPose.getX());
                telemetry.addData("Robot Y: ", robotPose.getY());
                telemetry.addData("Robot Heading: ", Math.toDegrees(robotPose.getHeading()));
            }

            timer.reset();

            robot.update();
            telemetry.update();
        }
    }
}
