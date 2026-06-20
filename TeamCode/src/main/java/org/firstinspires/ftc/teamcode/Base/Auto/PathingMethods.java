package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Auto.Misc.RunSide;
import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

public class PathingMethods {
    // 0 is the line furthest from the goal
    public static void intakeLine(RobotManager robot, RunSide runSide, double number, boolean clearGateWhileDriving) {
        robot.stopScoringCycle();
        robot.powerIntakeOn();
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

            if (runSide == RunSide.FAR_ZONE) {
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

            if (robot.autoPathingCancelFlagActive())
                return;
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

            if (robot.autoPathingCancelFlagActive())
                return;
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

                if (robot.autoPathingCancelFlagActive())
                    return;

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

                if (robot.autoPathingCancelFlagActive())
                    return;
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
                                .setVelocityConstraint(1000)
                        , true);

                if (robot.autoPathingCancelFlagActive())
                    return;
            }
        }

        robot.setMaxFollowerPower(1);
    }

    public static void intakeLine(RobotManager robot, RunSide runSide, double number) {
        intakeLine(robot, runSide, number, false);
    }

    public static void intakeGate(RobotManager robot, RunSide runSide, boolean initialCycle, boolean safeCycle) {
        Pose robotPose = robot.getPose();

        if (safeCycle) {
            Pose gatePose;
            Pose intakePose;

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

            if (robot.autoPathingCancelFlagActive())
                return;

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

                if (robot.autoPathingCancelFlagActive())
                    return;

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

                if (robot.autoPathingCancelFlagActive())
                    return;
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
                gateY = -73.7;
            }

            robot.setMaxFollowerPower(1);
            robot.addPathTimeout(1750);
            robot.powerIntakeOn();
            robot.runBlocking(
                    robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-12, -82),
                                            robot.getFixedPose(gateX, gateY)
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(gateHeading), .3)
                            .setTValueConstraint(.9)
                            .addParametricCallback(.05, robot::powerIntakeOff)
                            .addParametricCallback(.5, robot::powerIntakeOn)
                            .setHeadingConstraint(Math.PI)
                            .setVelocityConstraint(1000)
                            .setBrakingStrength(5)
                            .setTranslationalConstraint(1000)
                    , false);

            robot.breakFollowing(false);
            robot.setMaxFollowerPower(1);

            if (robot.autoPathingCancelFlagActive())
                return;

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

            robot.safeSleep(initialCycle ? 1000 : 1950);
            robot.breakFollowing(false);
            robot.clearPathTimeout();
        }
    }

    public static void clearGate(RobotManager robot, RunSide runSide, double waitTime) {
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

        if (robot.autoPathingCancelFlagActive())
            return;

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

    public static void intakeHumanPlayer(RobotManager robot, RunSide runSide, boolean sweepZone) {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.setIntakePower(1);
        robot.setMaxFollowerPower(1);
        robot.update();

        double wallX;
        double wallY;

        if (runSide == RunSide.CLOSE_ZONE) {
            wallY = -124;
        } else {
            wallY = -135;
        }


        if (robot.getAllianceSide() == AllianceSides.RED) {
            wallX = 21;
        } else {
            wallX = 19;
        }

        if (runSide == RunSide.CLOSE_ZONE) {
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
        } else if (runSide == RunSide.FAR_ZONE) {
            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierCurve(
                                            robotPose,
                                            robot.getFixedPose(-20, -110),
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

        if (robot.autoPathingCancelFlagActive())
            return;

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
            RunSide runSide,
            Pose startPose,
            boolean afterGate,
            boolean afterCloseLine,
            boolean shootOffTape,
            double intakeShutoffT,
            boolean initialCycle,
            double robotChassisHeading,
            double robotChassisHeadingEndT)
    {
        Pose shootingPosition;
        Pose robotPose = robot.getPose();

        robot.powerShooterOn();
        robot.powerIntakeOff();
        robot.setMaxFollowerPower(1);

        if (intakeShutoffT == -1) {
            intakeShutoffT = .1;
        }

        if (runSide == RunSide.CLOSE_ZONE) {
            if (initialCycle) {
                shootingPosition = robot.getFixedPose(-30, -40);
                Parameters.SHOOTER_GOAL_CLOSE_RED_AIM = new Pose(10, 0);

                robot.updateShooterParameters(shootingPosition);
                robot.addPathTimeout(2000);
                robot.runPassthrough(robot.pathBuilder()
                                .addPath(new Path(
                                        new BezierLine(
                                                startPose,
                                                shootingPosition
                                        )
                                ))
                                .setLinearHeadingInterpolation(robot.getFixedHeading(220), robot.getFixedHeading(robotChassisHeading), robotChassisHeadingEndT)
                                .setTValueConstraint(.9)
                                .setVelocityConstraint(5)
                );
                Parameters.SHOOTER_GOAL_CLOSE_RED_AIM = new Pose(0, 0);
            } else {
                if (shootOffTape) {
                    shootingPosition = robot.getFixedPose(-36, -20);
                } else {
                    shootingPosition = robot.getFixedPose(-30, -41);
                }

                robot.updateShooterParameters(shootingPosition);

                if (afterGate) {
                    robot.runPassthrough(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(-20, robotPose.getY() - 5),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robot.getFixedHeading(0), robot.getFixedHeading(robotChassisHeading), robotChassisHeadingEndT)
                                    .setTValueConstraint(.95)
                    );
                } else if (afterCloseLine) {
                    robot.runPassthrough(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierLine(
                                                    robotPose,
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(robotChassisHeading), robotChassisHeadingEndT)
                                    .setTValueConstraint(.95)
                    );
                } else {
                    robot.runPassthrough(robot.pathBuilder()
                                    .addPath(new Path(
                                            new BezierCurve(
                                                    robotPose,
                                                    robot.getFixedPose(-20, robotPose.getY() - 15),
                                                    shootingPosition
                                            )
                                    ))
                                    .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(robotChassisHeading), robotChassisHeadingEndT)
                                    .setTValueConstraint(.95)
                    );
                }
            }

            if (robot.autoPathingCancelFlagActive())
                return;

            robot.setIntakePower(1);

            robot.waitForCondition(() -> robot.getFollower().getCurrentTValue() >= .1);
            robot.powerIntakeOff();
            robot.waitForCondition(() -> robot.getFollower().getCurrentTValue() >= .2);
            robot.startScoringCycle(true);
            robot.waitForCondition(() -> robot.getFollower().getDistanceRemaining() <= Parameters.AUTO_SCORE_ERROR && (Parameters.AUTO_SCORE_MAX_VEL == -1 || robot.getFollower().getVelocity().getMagnitude() < Parameters.AUTO_SCORE_MAX_VEL));
            robot.setIntakePower(1);
            robot.safeSleep(Parameters.AUTO_SCORE_WAIT_TIME);

            robot.stopScoringCycle();
            robot.waitForPathEnd();
        } else if (runSide == RunSide.FAR_ZONE) {
            shootingPosition = robot.getFixedPose(-28, -122, 0);

            robot.updateShooterParameters(shootingPosition);

            robot.runBlocking(robot.pathBuilder()
                            .addPath(new Path(
                                    new BezierLine(
                                            robotPose,
                                            shootingPosition
                                    )
                            ))
                            .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(robotChassisHeading), robotChassisHeadingEndT)
                            .setTValueConstraint(.97)
                            .setVelocityConstraint(100)
            , false);

            robot.waitForShooter(2000);
            robot.startScoringCycle(true);
            robot.safeSleep(50);
            robot.powerIntakeOn();
            robot.safeSleep(1000);
            robot.stopScoringCycle();
            robot.powerIntakeOff();
        }
    }

    public static void park(RobotManager robot, RunSide runSide) {
        Pose robotPose = robot.getPose();

        robot.stopScoringCycle();
        robot.powerIntakeOff();
        robot.runBlocking(robot.pathBuilder()
                        .addPath(new Path(
                                new BezierLine(
                                        robotPose,
                                        robot.getFixedPose(-25, runSide == RunSide.FAR_ZONE ? -95 : -75)
                                )
                        ))
                        .setLinearHeadingInterpolation(robotPose.getHeading(), robot.getFixedHeading(0))
                        .setTValueConstraint(.98)
                        .setVelocityConstraint(1)
                , true);

        robot.safeSleep(350);
    }

}
