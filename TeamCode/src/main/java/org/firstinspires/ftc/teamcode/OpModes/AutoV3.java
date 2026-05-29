package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoProgram;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoStartSide;
import org.firstinspires.ftc.teamcode.Base.OpModeStates;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.ShootingStyle;

import java.util.ArrayList;

enum V3AutoDefault {
    EIGHTEEN_ARTIFACT,
    EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY,
    EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH,
    EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH_F1_BLUE,
    FIFTEEN_ARTIFACT,
    FIFTEEN_ARTIFACT_GATE_INTAKE,
    FIFTEEN_ARTIFACT_ALLIANCE_FRIENDLY,
    TWELVE_ARTIFACT,
    TWELVE_ARTIFACT_ALLIANCE_FRIENDLY,
    FAR_ZONE_6_ARTIFACT_FROM_HP,
    FAR_ZONE_9_ARTIFACT_FROM_HP_SM,
    FAR_ZONE_9_ARTIFACT_FROM_HP
}

@Autonomous(name = "Auto V3", group = "1", preselectTeleOp = "0: Main Teleop")
public class AutoV3 extends LinearOpMode {
    private boolean useDefault = true;
    private boolean robotStartIsSet = false;
    private V3AutoDefault defaultAuto = V3AutoDefault.EIGHTEEN_ARTIFACT;
    private RobotManager robot;
    private AutoStartSide startSide = AutoStartSide.CLOSE_ZONE;
    private AutoProgram internalProgram;
    private ArrayList<AutoCommandRepository.AutoCommand> commands = new ArrayList<>();

    private void setupFromDefault() {
        commands.clear();

        switch (defaultAuto) {
            case EIGHTEEN_ARTIFACT:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine(false));
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ClearGate(1000));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine(false));
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH_F1_BLUE:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ClearGate(1000));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine(false));
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case FIFTEEN_ARTIFACT:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ClearGate());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.Park());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case FIFTEEN_ARTIFACT_GATE_INTAKE:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case FIFTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case TWELVE_ARTIFACT:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.Park());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case TWELVE_ARTIFACT_ALLIANCE_FRIENDLY:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.Park());

                startSide = AutoStartSide.CLOSE_ZONE;
                break;
            case FAR_ZONE_6_ARTIFACT_FROM_HP:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));

                startSide = AutoStartSide.FAR_ZONE;
                break;
            case FAR_ZONE_9_ARTIFACT_FROM_HP_SM:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));

                startSide = AutoStartSide.FAR_ZONE;
                break;
            case FAR_ZONE_9_ARTIFACT_FROM_HP:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));

                startSide = AutoStartSide.FAR_ZONE;
                break;
        }
    }

    private String getDefaultUserLabel(V3AutoDefault ofType) {
        switch (ofType) {
            case EIGHTEEN_ARTIFACT:
                return "Close Zone: 18 Artifact Auto";
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                return "Close Zone: 18 Artifact Alliance Friendly Auto";
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH:
                return "Close Zone: 18 Artifact Alliance Friendly Auto with an Extra Gate Push";
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH_F1_BLUE:
                return "(Field 1 Blue Only) Close Zone: 18 Artifact Alliance Friendly Auto with an Extra Gate Push";
            case FIFTEEN_ARTIFACT:
                return "Close Zone: 15 Artifact Auto";
            case FIFTEEN_ARTIFACT_GATE_INTAKE:
                return "Close Zone: 15 Artifact Gate Intake Auto";
            case FIFTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                return "Close Zone: 15 Artifact Alliance Friendly Auto";
            case TWELVE_ARTIFACT:
                return "Close Zone: 12 Artifact Auto";
            case TWELVE_ARTIFACT_ALLIANCE_FRIENDLY:
                return "Close Zone: 12 Artifact Alliance Friendly Auto";
            case FAR_ZONE_9_ARTIFACT_FROM_HP:
                return "Far Zone: 9 Artifact (all from human player station)";
            case FAR_ZONE_9_ARTIFACT_FROM_HP_SM:
                return "Far Zone: 9 Artifact (from far spike mark and human player station)";
            case FAR_ZONE_6_ARTIFACT_FROM_HP:
                return "Far Zone: 6 Artifact (all from human player station)";
        }

        return "Unimplemented Default Auto";
    }

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

        int defaultSelection = defaultAuto.ordinal();
        boolean defaultUpdated = true;

        while (opModeInInit()) {
            if (gamepad1.yWasPressed())
                robot.setAllianceSide(robot.getAllianceSide() == AllianceSides.BLUE ? AllianceSides.RED : AllianceSides.BLUE);

            telemetry.addLine("Press Δ to change the robot's alliance.");
            telemetry.addLine("Press DPad Up/Down to change Autonomous Mode.");

            if (gamepad1.dpadUpWasPressed()) {
                defaultSelection++;
                defaultUpdated = true;
            } else if (gamepad1.dpadDownWasPressed()) {
                defaultSelection--;
                defaultUpdated = true;
            }

            if (defaultUpdated) {
                if (defaultSelection >= V3AutoDefault.values().length) {
                    defaultSelection = 0;
                } else if (defaultSelection < 0) {
                    defaultSelection = (V3AutoDefault.values().length - 1);
                }

                defaultAuto = V3AutoDefault.values()[defaultSelection];

                setupFromDefault();

                defaultUpdated = false;
                useDefault = true;
            }

            telemetry.addData("Alliance Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            telemetry.addData("Autonomous Default: ", getDefaultUserLabel(defaultAuto));
            telemetry.addData("Robot Heading: ", robot.getPose().getHeading());
            telemetry.addLine();
            telemetry.addLine("Auto Program:");
            telemetry.addData("Starting Side: ", startSide);

            for(AutoCommandRepository.AutoCommand command : commands) {
                telemetry.addData("Command", command.getUserLabel());
            }

            updateRobotStart();

            robot.update();
            telemetry.update();
        }

        if (useDefault && defaultAuto == V3AutoDefault.EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH_F1_BLUE) {
            robot.setAllianceSide(AllianceSides.BLUE);
        }

        waitForStart();
        updateRobotStart();

        internalProgram = new AutoProgram(robot);

        if (useDefault)
            setupFromDefault();

        internalProgram.setCommands(commands);
        internalProgram.execute(startSide, getStartPose());
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

    private Pose getStartPose() {
        Pose startPose = new Pose();

        if (startSide == AutoStartSide.CLOSE_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START;
        } else if (startSide == AutoStartSide.FAR_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START;
        }

        return startPose;
    }
}
