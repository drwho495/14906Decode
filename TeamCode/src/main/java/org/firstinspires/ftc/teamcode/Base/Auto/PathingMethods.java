package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterControlPolicy;

public class PathingMethods {
    // 0 is the line furthest from the goal
    public static void intakeLine(RobotManager robot, AutoStartSide startSide, double number, boolean clearGateWhileDriving) {
        robot.stopScoringCycle();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        Pose robotPose = robot.getPose();

        double wallX;
        double goalX;

        if (robot.getAllianceSide() == AllianceSides.RED) {
            wallX = 10;
            goalX = 9;
        } else {
            wallX = 9;
            goalX = 6;
        }

        if (number == 0) {
            Pose middlePose = new Pose(-25, -102);
            double endT = .7;

            if (startSide == AutoStartSide.FAR_ZONE) {
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
                            .setNoDeceleration()
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
                            .setNoDeceleration()
                            .setVelocityConstraint(1000)
                    , true);
        } else if (number == 2) {
            robot.turnTo(robot.getFixedHeading(0), Math.toRadians(35));

            robotPose = robot.getPose();

            robot.setMaxFollowerPower(.9);
            robot.addPathTimeout(2000);

            if (clearGateWhileDriving) {
                robot.runBlocking(
                        robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierCurve(
                                                robotPose,
                                                robot.getFixedPose(-15, -50),
                                                robot.getFixedPose(7, -60)
                                        )
                                ))
                                .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .4)
                                .setTValueConstraint(.99)
                                .setBrakingStrength(10)
                                .setVelocityConstraint(1000)
                        , true);

                robotPose = robot.getPose();

                robot.powerIntakeOff();
                robot.setMaxFollowerPower(.4);
                robot.runPassthrough(
                        robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                robotPose,
                                                robot.getFixedPose(15, -60)
                                        )
                                ))
                                .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .4)
                                .setTValueConstraint(.99)
                                .setNoDeceleration()
                                .setVelocityConstraint(1000)
                );

                robot.safeSleep(750);
                robot.breakFollowing(false);
            } else {
                robot.runBlocking(
                        robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierCurve(
                                                robotPose,
                                                robot.getFixedPose(-15, -50),
                                                robot.getFixedPose(10, -50)
                                        )
                                ))
                                .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0), .4)
                                .setTValueConstraint(.99)
//                                .setNoDeceleration()
                                .setVelocityConstraint(1000)
                        , true);
            }
        }

        robot.setMaxFollowerPower(1);
    }

    public static void intakeLine(RobotManager robot, AutoStartSide startSide, double number) {
        intakeLine(robot, startSide, number, false);
    }

    public static void intakeGate(RobotManager robot, AutoStartSide startSide, boolean initialCycle, boolean safeCycle) {
        Pose robotPose = robot.getPose();

        if (safeCycle) {
            Pose gatePose;
            Pose intakePose;

            // values are from test opmode
            if (robot.getAllianceSide() == AllianceSides.BLUE) {
                gatePose = robot.getFixedPose(18, -70, Math.toRadians(30));
                intakePose = robot.getFixedPose(16, -88, Math.toRadians(65));
            } else {
                gatePose = robot.getFixedPose(18, -70, Math.toRadians(30));
                intakePose = robot.getFixedPose(15, -88, Math.toRadians(65));
            }

            robot.setIntakePower(1);

            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-25, -95),
                                            robot.getFixedPose(18, -70)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robot.getFixedHeading(0), gatePose.getHeading())
                            .setTValueConstraint(.97)
                            .setVelocityConstraint(1000)
                            .setTimeoutConstraint(0)
                            .setTranslationalConstraint(10)
                            .setHeadingConstraint(Math.PI / 6)
                    , false);

            robot.safeSleep(350);

            for (int i = 0; i < 2; i++) {
                robotPose = robot.getPose();

                robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                        new BezierLine(
                                                robotPose,
                                                intakePose
                                        )
                                )
                        )
                        .setLinearHeadingInterpolation(robotPose.getHeading(), intakePose.getHeading())
                        .setTValueConstraint(.95)
                        .setVelocityConstraint(1000)
                        .setTimeoutConstraint(0)
                        .setTranslationalConstraint(10)
                        .setNoDeceleration()
                        .setHeadingConstraint(Math.PI / 6)
                );

                robotPose = robot.getPose();

                robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                        new BezierLine(
                                                robotPose,
                                                gatePose
                                        )
                                )
                        )
                        .setLinearHeadingInterpolation(robotPose.getHeading(), gatePose.getHeading())
                        .setTValueConstraint(.95)
                        .setVelocityConstraint(1000)
                        .setTimeoutConstraint(0)
                        .setTranslationalConstraint(10)
                        .setNoDeceleration()
                        .setHeadingConstraint(Math.PI / 6)
                );
            }
        } else {
            double gateHeading = 25;
            double gateX;
            double gateY;

            if (robot.getAllianceSide() == AllianceSides.RED) {
                gateX = 11;
                gateY = -76;
            } else {
                gateX = 11;
                gateY = -74;
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
                            .setTValueConstraint(.9)
                            .setHeadingConstraint(Math.PI)
                            .setVelocityConstraint(1000)
                            .setBrakingStrength(5)
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
                                            robot.getFixedPose(19, gateY)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateHeading), 1)
                            .setNoDeceleration()
                            .setTValueConstraint(1)
            );

            robot.safeSleep(initialCycle ? 1000 : 1350);
            robot.breakFollowing(false);
            robot.clearPathTimeout();
        }
    }

    public static void clearGate(RobotManager robot, AutoStartSide startSide, double waitTime) {
        Pose robotPose = robot.getPose();

        double pushHeading = robot.getFixedHeading(0);

        robot.powerIntakeOff();
        robot.setMaxFollowerPower(1);
        robot.addPathTimeout(900);

        ElapsedTime timer = new ElapsedTime();
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

        robot.safeSleep(timer, waitTime);
        robot.breakFollowing();
        robot.setMaxFollowerPower(1);
    }

    public static void intakeHumanPlayer(RobotManager robot, AutoStartSide startSide, boolean sweepZone) {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        double wallX;
        double wallY;

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            wallY = -124;
        } else {
            wallY = -130;
        }


        if (robot.getAllianceSide() == AllianceSides.RED) {
            wallX = 21;
        } else {
            wallX = 19;
        }

        if (startSide == AutoStartSide.CLOSE_ZONE) {
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
        } else if (startSide == AutoStartSide.FAR_ZONE) {
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-10, -90),
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

        robotPose = robot.getPose();

        robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        robotPose,
                                        robot.getFixedPose(wallX - 20, wallY + 2),
                                        robot.getFixedPose(wallX - 12, wallY + 15),
                                        robot.getFixedPose(wallX - 10, wallY + 40)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(30), .2)
                        .setTValueConstraint(.9)
                        .setVelocityConstraint(100)
                        .setTimeoutConstraint(0),
                true
        );

        robot.safeSleep(100);
    }

    public static void scoreArtifacts(
            RobotManager robot,
            AutoStartSide startSide,
            Pose startPose,
            boolean afterGate,
            boolean afterCloseLine,
            boolean shootOffTape,
            double intakeShutoffT,
            boolean initialCycle)
    {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        if (robot.getShooterControlPolicy() == ShooterControlPolicy.MANUAL) {
            if (startSide == AutoStartSide.FAR_ZONE) {
                robot.setShooterVelocity(Parameters.SHOOTER_FAR_ZONE_VELOCITY);
                robot.setHoodServoPos(Parameters.SHOOTER_FAR_ZONE_HOOD_ANGLE);
            } else {
                if (shootOffTape) {
                    if (robot.getAllianceSide() == AllianceSides.RED) {
                        robot.setShooterVelocity(3350);
                        robot.setHoodServoPos(20);
                    } else {
                        robot.setShooterVelocity(3300);
                        robot.setHoodServoPos(20);
                    }
                } else {
                    robot.setShooterVelocity(3350);
                    robot.setHoodServoPos(20);
                }
            }

            robot.update();
        }

        robot.powerShooterOn();
        robot.setMaxFollowerPower(1);

        if (intakeShutoffT == -1) {
            intakeShutoffT = .1;
        }

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            if (initialCycle) {
                robot.startScoringCycle(); // make sure that servo opens!
                robot.update();

                shootingPosition = robot.getFixedPose(-27, -40);

//                robot.updateShooterParameters(shootingPosition);

                robot.addPathTimeout(2000);
                robot.runBlocking(robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                startPose,
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
                            robot.getAllianceSide() == AllianceSides.RED ? -30 : -29,
                            -28,
                            Math.toRadians(180)
                    );
                } else {
                    shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
                }

//                robot.updateShooterParameters(shootingPosition);

                if (afterGate) {
                    robot.runBlocking(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(5, robotPose.getY() - 5),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robot.getFixedHeading(0), robot.getHeadingToGoal(shootingPosition), 1, .3)
                                    .addParametricCallback(intakeShutoffT, robot::powerIntakeOff)
                                    .setTValueConstraint(.95)
                            , false);
                } else if (afterCloseLine) {
                    robot.runBlocking(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierLine(
                                                    robotPose,
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                    .addParametricCallback(intakeShutoffT, robot::powerIntakeOff)
                                    .setTValueConstraint(.95)
                            , false);
                } else {
                    robot.runBlocking(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(-20, robotPose.getY() - 15),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getHeadingToGoal(shootingPosition))
                                    .addParametricCallback(intakeShutoffT, robot::powerIntakeOff)
                                    .setTValueConstraint(.95)
                            , false);
                }
            }
        } else if (startSide == AutoStartSide.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-30, -115, 0);

//            robot.updateShooterParameters(shootingPosition);

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
        robot.setIntakePower(1);

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            if (initialCycle) {
                robot.waitForShooter(500);
            } else {
                robot.waitForShooter(150);
            }
        } else if (startSide == AutoStartSide.FAR_ZONE) {
            if (initialCycle) {
                robot.waitForShooter();
            } else {
                robot.waitForShooter(600);
            }

            robot.waitForHeadingCorrection(.75, 300, 1);
        }

        robot.startScoringCycle();
        robot.setIntakePower(1);

        robot.safeSleep(650);

        robot.stopScoringCycle();
        robot.safeSleep(50);

        robot.update();
    }

    public static void park(RobotManager robot, AutoStartSide startSide) {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        robotPose,
                                        robot.getFixedPose(-25, startSide == AutoStartSide.FAR_ZONE ? -95 : -75)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                        .setTValueConstraint(.98)
                        .setVelocityConstraint(1)
                , true);

        robot.safeSleep(350);
    }

}
