package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoStartSide;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.ShootingStyle;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

enum AutoType {
    FIFTEEN_ARTIFACT,
    FIFTEEN_ALLIANCE_FRIENDLY,
    TWELVE_ARTIFACT,
    EIGHTEEN_ARTIFACT,
    EIGHTEEN_ALLIANCE_FRIENDLY,
    CLOSE_ZONE_9,
    CLOSE_ZONE_6,
    GATE_INTAKE_DEBUG,
}

@Autonomous(name = "Auto V2", group = "1", preselectTeleOp = "0: Main Teleop")
public class AutoV2 extends LinearOpMode {
    private RobotManager robot;
    private AutoStartSide autoStartPos = AutoStartSide.CLOSE_ZONE;
    private boolean clearGate = true;
    private boolean grabFromLine0 = true;
    private boolean grabFromHumanPlayer = true;
    private int grabFromGateCycles = 0;
    private boolean delayedSecondGateCycle = false;
    private boolean farGrabFromLine = true;
    private boolean robotStartIsSet = false;
    private boolean shootLastCycleOffTape = false;
    private boolean debugPark = false;
    private AutoType autoType = Parameters.ROBOT == 0 ? AutoType.FIFTEEN_ARTIFACT : AutoType.EIGHTEEN_ARTIFACT;

    private ElapsedTime timer = new ElapsedTime();
    private ArrayList<String> headingErrors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);

        robot.setState(OpModeStates.INTAKE_SCORE);
        robot.setShooterControlPolicy(ShooterControlPolicy.MANUAL);
        robot.enableAutoTransferStop();
        robot.disableDebugPrinting();
        robot.disableVelocityCompensation();
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.disableHoodCompensation();
        robot.disableOnlyShootInZone();
        robot.disableWaitForVelocityToShoot();
        robot.disablePoweredHold();

        robot.initialise();
        robot.recalibrateIMU();

        robot.startScoringCycle();
        robot.update();

        int autoTypeSelection = autoType.ordinal();

        while (opModeInInit()) {
            robot.powerOffShooter();

            if (gamepad1.yWasPressed())
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.BLUE ? AllianceSides.RED : AllianceSides.BLUE);

            telemetry.addLine("Press Δ to change the robot's alliance.");
            telemetry.addLine("Press DPad Up/Down to change Autonomous Mode.");

            if (gamepad1.dpadUpWasPressed()) {
                autoTypeSelection++;
            } else if (gamepad1.dpadDownWasPressed()) {
                autoTypeSelection--;
            }

            if (autoTypeSelection >= AutoType.values().length) {
                autoTypeSelection = 0;
            } else if (autoTypeSelection < 0) {
                autoTypeSelection = (AutoType.values().length - 1);
            }

            autoType = AutoType.values()[autoTypeSelection];
            telemetry.addData("Alliance Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            telemetry.addData("Autonomous Mode: ", autoType);
            telemetry.addData("Robot Heading: ", robot.getPose().getHeading());

            updateAutoSettings();
            updateRobotStart();

            robot.update();
            telemetry.update();
        }

        updateAutoSettings();
        updateRobotStart();

        robot.enableDebugPrinting();

        waitForStart();

        int gateCycleNumber = 0;

        robot.update();
        robot.getPose();

        timer.reset();
        Parameters.LAST_ALLIANCE_SIDE = robot.getAllianceSide();

        scoreArtifacts(0, false, false);

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            if (grabFromGateCycles == 0) {
                if (grabFromLine0) {
                    lineIntake(0);
                    scoreArtifacts(1, false, false);
                }
                lineIntake(1);
                scoreArtifacts(2, false, false);
                lineIntake(2);
                scoreArtifacts(3, false, false);

                if (grabFromHumanPlayer && clearGate) {
                    humanPlayerIntake();
                    scoreArtifacts(3, false, false, .4);
                }
            } else if (grabFromGateCycles > 0) {
                lineIntake(1);
                scoreArtifacts(2, false, false);

                if (delayedSecondGateCycle) {
                    gateIntake(gateCycleNumber++);
                    scoreArtifacts(3, true, false);

                    grabFromGateCycles--;
                } else {
                    for (int i = 0; i < grabFromGateCycles; i++) {
                        gateIntake(gateCycleNumber++);
                        scoreArtifacts(3, true, false);
                    }
                }

                if (grabFromLine0) {
                    lineIntake(0);
                    scoreArtifacts(1, false, false);
                }

                if (grabFromHumanPlayer) {
                    humanPlayerIntake();
                    scoreArtifacts(3, false, false, .4);
                }

                lineIntake(2);

                if (clearGate || grabFromGateCycles != 0) {
                    scoreArtifacts(3, false, shootLastCycleOffTape && !delayedSecondGateCycle);
                }

                if (delayedSecondGateCycle && grabFromGateCycles > 0) {
                    for (int i = 0; i < grabFromGateCycles; i++) {
                        gateIntake(gateCycleNumber++);
                        scoreArtifacts(3, true, shootLastCycleOffTape);
                    }
                }
            }

            if (!shootLastCycleOffTape) {
                park();
            } else {
                robot.breakFollowing(false);
            }

            if (debugPark) {
                robot.powerOffShooter();
                robot.powerOffIntake();

                robot.safeSleep(2000);

                Pose robotPose = robot.getPose();
                Pose startPose = getStartPose();

                robot.setMaxFollowerPower(.5);
                robot.runBlocking(
                        robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                robotPose,
                                                startPose
                                        )
                                ))
                                .setLinearHeadingInterpolation(robotPose.getHeading(), startPose.getHeading(), 1)
                                .setTValueConstraint(.92)
                );
            }
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            int shootOffset = 0;

            if (farGrabFromLine) {
                shootOffset = 1;

                lineIntake(0);
                scoreArtifacts(1, false, false);
            }

            humanPlayerIntake();
            scoreArtifacts(1 + shootOffset, false, false, .2);

            robot.powerOffShooter();
            robot.powerOffIntake();

            while (opModeIsActive() && timer.time(TimeUnit.SECONDS) <= 27) {
                robot.update();
            }

            park();
        }

        robot.powerOffShooter();
        robot.powerOffIntake();
        robot.update();

        double time = ((double) timer.time(TimeUnit.MILLISECONDS)) / 1000;

        while (opModeIsActive()) {
            telemetry.addData("Time: ", time);
            telemetry.addData("Heading Errors: ", String.join(", ", headingErrors));
            telemetry.update();
        }
    }

    private Pose getStartPose() {
        Pose startPose = new Pose();

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START;
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START;
        }

        return startPose;
    }

    private void updateRobotStart() {
        if (opModeIsActive()) {
            if (robotStartIsSet) return;
            robotStartIsSet = true;
        } else {
            robotStartIsSet = false;
        }

        robot.setPose(getStartPose());
    }

    private void updateAutoSettings() {
        switch (autoType) {
            case TWELVE_ARTIFACT:
                clearGate = true;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 0;
                delayedSecondGateCycle = false;
                grabFromLine0 = true;
                shootLastCycleOffTape = false;
                debugPark = false;
                break;
            case FIFTEEN_ARTIFACT:
                clearGate = true;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = true;
                grabFromGateCycles = 0;
                delayedSecondGateCycle = false;
                grabFromLine0 = true;
                shootLastCycleOffTape = false;
                debugPark = false;
                break;
            case FIFTEEN_ALLIANCE_FRIENDLY:
                clearGate = false;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 2;
                delayedSecondGateCycle = true;
                grabFromLine0 = false;
                shootLastCycleOffTape = false;
                debugPark = false;
                break;
            case EIGHTEEN_ARTIFACT:
                clearGate = false;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 2;
                delayedSecondGateCycle = false;
                grabFromLine0 = true;
                shootLastCycleOffTape = true;
                debugPark = false;
                break;
            case EIGHTEEN_ALLIANCE_FRIENDLY:
                clearGate = false;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 3;
                delayedSecondGateCycle = false;
                grabFromLine0 = false;
                shootLastCycleOffTape = true;
                debugPark = false;
                break;
            case GATE_INTAKE_DEBUG:
                clearGate = false;
                autoStartPos = AutoStartSide.CLOSE_ZONE;
                grabFromHumanPlayer = false;
                grabFromGateCycles = 6;
                delayedSecondGateCycle = false;
                grabFromLine0 = false;
                shootLastCycleOffTape = true;
                debugPark = true;
                break;
            case CLOSE_ZONE_9:
                clearGate = false;
                autoStartPos = AutoStartSide.FAR_ZONE;
                grabFromHumanPlayer = true;
                grabFromGateCycles = 0;
                delayedSecondGateCycle = false;
                farGrabFromLine = true;
                shootLastCycleOffTape = true;
                debugPark = false;
                break;
            case CLOSE_ZONE_6:
                clearGate = false;
                autoStartPos = AutoStartSide.FAR_ZONE;
                grabFromHumanPlayer = true;
                grabFromGateCycles = 0;
                delayedSecondGateCycle = false;
                farGrabFromLine = false;
                shootLastCycleOffTape = true;
                debugPark = false;
                break;
        }

        if (robot.getAllianceSide() == AllianceSides.BLUE) {
            robot.setGoalOffset(4);
        }

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            robot.setTransferSpeed(1);
            robot.resetGoalOffset();
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            robot.setTransferSpeed(.2);
            robot.setGoalOffset(-8);
        }
    }

    // 0 is the line furthest from the goal
    private void lineIntake(double number) {
        robot.stopScoringCycle();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        Pose robotPose = robot.getPose();

        double wallX;
        double goalX;

        if (Parameters.ROBOT == 0) {
            if (robot.getAllianceSide() == AllianceSides.RED) {
                wallX = 17;
                goalX = 9;
            } else {
                wallX = 16;
                goalX = 7;
            }
        } else {
            if (robot.getAllianceSide() == AllianceSides.RED) {
                wallX = 17;
                goalX = 9;
            } else {
                wallX = 14;
                goalX = 6;
            }
        }

        if (number == 0) {
            Pose middlePose = new Pose(-25, -102);
            double endT = .7;

            if (autoStartPos == AutoStartSide.FAR_ZONE) {
                middlePose = new Pose(-47, -95);
                endT = .4;
            }

            robot.addPathTimeout(2000);
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(middlePose),
                                            robot.getFixedPose(wallX, -98)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), endT)
                            .setTValueConstraint(.99)
                            .setVelocityConstraint(1000)
                    , true);
        } else if (number == 1) {
            robot.addPathTimeout(2000);
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-25, -75),
                                            robot.getFixedPose(wallX, -77)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .6)
                            .setTValueConstraint(.99)
                            .setVelocityConstraint(1000)
                    , true);
        } else if (number == 2) {
            robot.turnTo(robot.getFixedHeading(0), Math.toRadians(35));

            robotPose = robot.getPose();

            robot.setMaxFollowerPower(.9);

            robot.addPathTimeout(2000);
            robot.runBlocking(
                    robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            robotPose,
                                            robot.getFixedPose(goalX, -50)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .4)
                            .setTValueConstraint(.99)
                            .setVelocityConstraint(1000)
                    , true);

            robot.setMaxFollowerPower(1);
        }

        robot.setMaxFollowerPower(1);

        if (number == 1) {
            if (clearGate) {
                clearGate();
            }
        }
    }

    private void gateIntake(int cycleNumber) {
        Pose robotPose = robot.getPose();

        double gateHeading = 25;
        double gateX;
        double gateY;

        if (robot.getAllianceSide() == AllianceSides.RED) {
            gateX = 12;
            gateY = -75.9;
        } else {
            gateX = 10;
            gateY = -73.3;
        }

        robot.setMaxFollowerPower(1);
        robot.setIntakePower(1);
        robot.addPathTimeout(1750);
        robot.runBlocking(
                robot.pathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        robotPose,
                                        robot.getFixedPose(-9, -78),
                                        robot.getFixedPose(gateX, gateY)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateHeading), .3)
                        .setTValueConstraint(.94)
                        .setHeadingConstraint(Math.PI)
                        .setVelocityConstraint(1000)
                        .setTranslationalConstraint(1000)
                , false);

        robot.breakFollowing(false);
        robot.setMaxFollowerPower(1);

        robotPose = robot.getPose();
        robot.clearPathTimeout();
        robot.runPassthrough(
                robot.pathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        robotPose,
                                        robot.getFixedPose(18, gateY)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateHeading), 1)
                        .setTValueConstraint(1)
        );

        robot.safeSleep(cycleNumber == 0 ? 1000 : 1250);
        robot.breakFollowing(false);
        robot.clearPathTimeout();
    }

    private void clearGate() {
        Pose robotPose = robot.getPose();

        double pushHeading = robot.getFixedHeading(0);

        robot.powerOffIntake();
        robot.setMaxFollowerPower(1);
        robot.addPathTimeout(900);

        timer.reset();

        robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        robotPose,
                                        robot.getFixedPose(-6, -70),
                                        robot.getFixedPose(9, -63)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
                        .setTValueConstraint(.99)
                , false);

        robotPose = robot.getPose();

        robot.setMaxFollowerPower(.3);
        robot.runPassthrough(robot.pathBuilder()
                .addPath(new Path(
                        new BezierLine(
                                robotPose,
                                robot.getFixedPose(18, -63)
                        )
                ))
                .setLinearHeadingInterpolation(robotPose.getHeading(), pushHeading)
                .setTValueConstraint(.99)
        );

        robot.safeSleep(timer, 2500);
        robot.breakFollowing();
        robot.setMaxFollowerPower(1);
    }

    private void humanPlayerIntake() {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        double wallX;
        double wallY;

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            wallY = -124;
        } else {
            wallY = -126;
        }

        if (Parameters.ROBOT == 0) {
            if (robot.getAllianceSide() == AllianceSides.RED) {
                wallX = 21;
            } else {
                wallX = 20;
            }
        } else {
            if (robot.getAllianceSide() == AllianceSides.RED) {
                wallX = 21;
            } else {
                wallX = 19;
            }
        }

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-38, wallY + 10),
                                            robot.getFixedPose(wallX, wallY)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), 1)
                            .setTValueConstraint(.95)
                            .setVelocityConstraint(100)
                            .setTimeoutConstraint(0),
                    false
            );
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-10, -80f),
                                            robot.getFixedPose(wallX, wallY)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .5)
                            .setTValueConstraint(.9)
                            .setVelocityConstraint(100)
                            .setTimeoutConstraint(0),
                    true
            );
        }

        robot.safeSleep(100);
    }

    private void scoreArtifacts(double cycleNumber, boolean afterGate, boolean shootOffTape) {
        scoreArtifacts(cycleNumber, afterGate, shootOffTape, -1);
    }

    private void scoreArtifacts(double cycleNumber, boolean afterGate, boolean shootOffTape, double intakeShutoffT) {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        robot.powerOnShooter();
        robot.setMaxFollowerPower(1);

        if (intakeShutoffT == -1) {
            intakeShutoffT = .1;
        }

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            if (cycleNumber == 0) {
                robot.startScoringCycle(); // make sure that servo opens!
                robot.update();

                shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
                robot.updateShooterParameters(shootingPosition);

                robot.addPathTimeout(2000);
                robot.runBlocking(robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                getStartPose(),
                                                shootingPosition
                                        )
                                ))
                                .setLinearHeadingInterpolation(robot.getFixedHeading(220), robot.getHeadingToGoal(shootingPosition))
                                .setTValueConstraint(.9)
                                .setVelocityConstraint(5)
                        , false);
            } else {
                if (shootOffTape) {
                    shootingPosition = robot.getFixedPose(
                            robot.getAllianceSide() == AllianceSides.RED ? -28 : -32,
                            -28, Math.toRadians(180)
                    );
                } else {
                    shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
                }

                robot.updateShooterParameters(shootingPosition);

                if (cycleNumber == 1) {
                    robot.runBlocking(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(-20, robotPose.getY() - 15),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                    .addParametricCallback(intakeShutoffT, () -> robot.powerOffIntake())
                                    .setTValueConstraint(.95)
                            , false);
                } else if (cycleNumber == 2) {
                    robot.runBlocking(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(-20, robotPose.getY() - 15),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                    .addParametricCallback(intakeShutoffT, () -> robot.powerOffIntake())
                                    .setTValueConstraint(.95)
                            , false);
                } else if (cycleNumber == 3) {
                    if (afterGate) {
                        robot.runBlocking(robot.pathBuilder()
                                        .addPath(new Path(
                                                new BezierCurve(
                                                        robotPose,
                                                        robot.getFixedPose(5, robotPose.getY() - 10),
                                                        shootingPosition
                                                )
                                        ))
                                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                        .addParametricCallback(intakeShutoffT, () -> robot.powerOffIntake())
                                        .setTValueConstraint(.95)
                                , false);
                    } else {
                        robot.runBlocking(robot.pathBuilder()
                                        .addPath(new Path(
                                                new BezierLine(
                                                        robotPose,
                                                        shootingPosition
                                                )
                                        ))
                                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                        .addParametricCallback(intakeShutoffT, () -> robot.powerOffIntake())
                                        .setTValueConstraint(.95)
                                , false);
                    }
                }
            }
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-35, -115, 0);

            robot.setShooterVelocity(Parameters.SHOOTER_FAR_ZONE_VELOCITY);
            robot.setHoodServoPos(Parameters.SHOOTER_FAR_ZONE_HOOD_ANGLE);
            robot.update();

            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            robotPose,
                                            shootingPosition
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                            .setTValueConstraint(.97)
                            .setVelocityConstraint(100)
                    , false);
        }

        robot.stopAndAim();

        if (autoStartPos == AutoStartSide.CLOSE_ZONE) {
            robot.setIntakePower(1); // this probably shouldnt be here
            if (cycleNumber == 0) {
                robot.waitForShooter(1200);
            } else {
                robot.waitForShooter(150);
            }
        } else if (autoStartPos == AutoStartSide.FAR_ZONE) {
            if (cycleNumber == 0) {
                robot.waitForShooter();
            } else {
                robot.waitForShooter(600);
            }

            robot.waitForHeadingCorrection(.75, 300, 1);
        }

        robot.setIntakePower(1);
        robot.safeSleep(25);
        robot.startScoringCycle();
        robot.setIntakePower(1);
        headingErrors.add(Double.toString(Math.toDegrees(robot.getFollower().getHeadingError())));

        robot.safeSleep(autoStartPos == AutoStartSide.CLOSE_ZONE ? 750 : 1500);

        robot.stopScoringCycle();
        robot.safeSleep(50);

        robot.update();
    }

    private void park() {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        robotPose,
                                        robot.getFixedPose(-25, autoStartPos == AutoStartSide.FAR_ZONE ? -95 : -75)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                        .setTValueConstraint(.98)
                        .setVelocityConstraint(1)
                , true);

        robot.safeSleep(350);
    }
}
