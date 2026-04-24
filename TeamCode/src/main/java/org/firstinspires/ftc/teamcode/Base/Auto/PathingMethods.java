package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShooterControlPolicy;

public class PathingMethods {
    // 0 is the line furthest from the goal
    public static void lineIntake(RobotManager robot, AutoStartSide startSide, double number) {
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
    }

    public static void intakeGate(RobotManager robot, AutoStartSide startSide, boolean initialCycle) {
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

        robot.safeSleep(initialCycle ? 1000 : 1250);
        robot.breakFollowing(false);
        robot.clearPathTimeout();
    }

    public static void clearGate(RobotManager robot, AutoStartSide startSide) {
        Pose robotPose = robot.getPose();

        double pushHeading = robot.getFixedHeading(0);

        robot.powerOffIntake();
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

        robot.safeSleep(timer, 2500);
        robot.breakFollowing();
        robot.setMaxFollowerPower(1);
    }

    public static void intakeHumanPlayer(RobotManager robot, AutoStartSide startSide) {
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

    public static void scoreArtifacts(
            RobotManager robot,
            AutoStartSide startSide,
            Pose startPose,
            boolean afterGate,
            boolean afterCloseLine,
            boolean shootOffTape,
            double intakeShutoffT,
            boolean initialCycle
    ) {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        if (robot.getShooterControlPolicy() == ShooterControlPolicy.MANUAL && startSide == AutoStartSide.FAR_ZONE) {
            robot.setShooterVelocity(Parameters.SHOOTER_FAR_ZONE_VELOCITY);
            robot.setHoodServoPos(Parameters.SHOOTER_FAR_ZONE_HOOD_ANGLE);
            robot.update();
        }

        robot.powerOnShooter();
        robot.setMaxFollowerPower(1);

        if (intakeShutoffT == -1) {
            intakeShutoffT = .1;
        }

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            if (initialCycle) {
                robot.startScoringCycle(); // make sure that servo opens!
                robot.update();

                shootingPosition = robot.getFixedPose(-23, -37);
                robot.updateShooterParameters(shootingPosition);

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
                            robot.getAllianceSide() == AllianceSides.RED ? -28 : -32,
                            -28,
                            Math.toRadians(180)
                    );
                } else {
                    shootingPosition = robot.getFixedPose(-30, -40, Math.toRadians(180));
                }

                robot.updateShooterParameters(shootingPosition);

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
                                    .addParametricCallback(intakeShutoffT, robot::powerOffIntake)
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
                                    .addParametricCallback(intakeShutoffT, robot::powerOffIntake)
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
                                    .addParametricCallback(intakeShutoffT, robot::powerOffIntake)
                                    .setTValueConstraint(.95)
                            , false);
                }
            }
        } else if (startSide == AutoStartSide.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-35, -115, 0);

            if (robot.getShooterControlPolicy() != ShooterControlPolicy.MANUAL)
                robot.updateShooterParameters(shootingPosition);

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

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            robot.setIntakePower(1); // this probably shouldnt be here
            if (initialCycle) {
                robot.waitForShooter(1200);
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

        robot.setIntakePower(1);
        robot.safeSleep(25);
        robot.startScoringCycle();
        robot.setIntakePower(1);

        robot.safeSleep(750);

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
