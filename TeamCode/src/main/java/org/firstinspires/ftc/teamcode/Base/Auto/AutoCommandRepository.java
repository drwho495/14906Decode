package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Base.Auto.Misc.RunSide;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

public class AutoCommandRepository {
    public enum AutoCommandTypes {
        INTAKE_FAR_LINE,
        INTAKE_MID_LINE,
        INTAKE_CLOSE_LINE,
        SCORE_ARTIFACTS,
        SCORE_ARTIFACTS_AND_PARK,
        PARK,
        INTAKE_GATE,
        INTAKE_HUMAN_PLAYER,
        CLEAR_GATE,
    }

    public static abstract class AutoCommand {
        public abstract AutoCommandTypes getType();

        public String getUserLabel() {
            return AutoCommandRepository.getUserLabel(getType());
        }

        public abstract void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand);
    }

    public static String getUserLabel(AutoCommandTypes ofType) {
        switch (ofType) {
            case INTAKE_FAR_LINE:
                return "Intake from the Far Line";
            case INTAKE_MID_LINE:
                return "Intake from the Mid Line";
            case INTAKE_CLOSE_LINE:
                return "Intake from the Close Line";
            case SCORE_ARTIFACTS:
                return "Score Artifacts";
            case SCORE_ARTIFACTS_AND_PARK:
                return "Score Artifacts and Park";
            case PARK:
                return "Park";
            case INTAKE_GATE:
                return "Intake from the Gate";
            case INTAKE_HUMAN_PLAYER:
                return "Intake from the Human Player";
            case CLEAR_GATE:
                return "Clear All Artifacts from the Gate";
        }

        return "Unimplemented";
    }

    public static AutoCommand makeCommand(AutoCommandTypes ofType) {
        switch (ofType) {
            case INTAKE_FAR_LINE:
                return new IntakeFarLine();
            case INTAKE_MID_LINE:
                return new IntakeMidLine();
            case INTAKE_CLOSE_LINE:
                return new IntakeCloseLine();
            case SCORE_ARTIFACTS:
                return new ScoreArtifacts(false);
            case SCORE_ARTIFACTS_AND_PARK:
                return new ScoreArtifactsAndPark();
            case PARK:
                return new Park();
            case INTAKE_GATE:
                return new IntakeGate(false);
            case INTAKE_HUMAN_PLAYER:
                return new IntakeHumanPlayer();
            case CLEAR_GATE:
                return new ClearGate();
        }

        return null;
    }

    public static class IntakeFarLine extends AutoCommand {
        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.INTAKE_FAR_LINE;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.intakeLine(
                    robot,
                    runSide,
                    0
            );
        }
    }

    public static class IntakeMidLine extends AutoCommand {
        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.INTAKE_MID_LINE;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.intakeLine(
                    robot,
                    runSide,
                    1
            );
        }
    }

    public static class IntakeCloseLine extends AutoCommand {
        private boolean clearGateWhileMoving = false;

        public IntakeCloseLine(boolean clearGateWhileMoving) {
            this.clearGateWhileMoving = clearGateWhileMoving;
        }

        public IntakeCloseLine() {
        }

        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.INTAKE_CLOSE_LINE;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.intakeLine(
                    robot,
                    runSide,
                    2,
                    clearGateWhileMoving
            );
        }
    }

    public static class ScoreArtifacts extends AutoCommand {
        private final boolean initialCycle;

        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.SCORE_ARTIFACTS;
        }

        public ScoreArtifacts(boolean initialCycle) {
            this.initialCycle = initialCycle;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            double intakeEndT = .05;
            double chassisHeading = -90;
            double chassisHeadingEndT = 1;

            if (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_HUMAN_PLAYER) {
                intakeEndT = .5;
            }

            if (runSide == RunSide.CLOSE_ZONE) {
                if (nextCommand != null) {
                    AutoCommandTypes type = nextCommand.getType();

                    if (type == AutoCommandTypes.INTAKE_GATE) {
                        chassisHeading = -45;
                        chassisHeadingEndT = .6;
                    } else if (type == AutoCommandTypes.INTAKE_CLOSE_LINE) {
                        chassisHeading = 0;
                        chassisHeadingEndT = .3;
                    }
                }
            } else if (runSide == RunSide.FAR_ZONE) {
                chassisHeading = 0;
            }

            PathingMethods.scoreArtifacts(
                    robot,
                    runSide,
                    startPose,
                    (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_GATE),
                    (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_CLOSE_LINE),
                    false,
                    intakeEndT,
                    this.initialCycle,
                    chassisHeading,
                    chassisHeadingEndT
            );
        }
    }

    public static class ScoreArtifactsAndPark extends AutoCommand {
        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.SCORE_ARTIFACTS_AND_PARK;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            double intakeEndT = .4;

            if (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_HUMAN_PLAYER) {
                intakeEndT = .5;
            }

            PathingMethods.scoreArtifacts(
                    robot,
                    runSide,
                    startPose,
                    (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_GATE),
                    (lastCommand != null && lastCommand.getType() == AutoCommandTypes.INTAKE_CLOSE_LINE),
                    true,
                    intakeEndT,
                    false,
                    -45,
                    1
            );
        }
    }

    public static class IntakeGate extends AutoCommand {
        private boolean initialCycle;

        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.INTAKE_GATE;
        }

        public IntakeGate(boolean initialCycle) {
            this.initialCycle = initialCycle;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.intakeGate(
                    robot,
                    runSide,
                    initialCycle,
                    false
            );
        }
    }

    public static class IntakeHumanPlayer extends AutoCommand {
        private boolean sweepZone;

        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.INTAKE_HUMAN_PLAYER;
        }

        public IntakeHumanPlayer(boolean sweep) {
            this.sweepZone = sweep;
        }

        public IntakeHumanPlayer() {
            this.sweepZone = false;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.intakeHumanPlayer(
                    robot,
                    runSide,
                    sweepZone
            );
        }
    }

    public static class Park extends AutoCommand {
        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.PARK;
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.park(
                    robot,
                    runSide
            );
        }
    }

    public static class ClearGate extends AutoCommand {
        private double waitTime = 2500;
        @Override
        public AutoCommandTypes getType() {
            return AutoCommandTypes.CLEAR_GATE;
        }

        public ClearGate(double waitTime) {
            this.waitTime = waitTime;
        }

        public ClearGate() {
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, RunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            PathingMethods.clearGate(
                    robot,
                    runSide,
                    waitTime
            );
        }
    }
}