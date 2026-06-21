package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Auto.Misc.RunSide;
import org.firstinspires.ftc.teamcode.Base.Misc.AllianceSides;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoProgram;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterAimPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.TurretBacklashPolicy;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Misc.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Misc.ShootingStyle;

import java.util.ArrayList;

enum V3AutoDefault {
    EIGHTEEN_ARTIFACT,
    EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY,
    F1B_EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY,
    EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH,
    TWENTY_ONE_ARTIFACT_ALLIANCE_FRIENDLY,
    FIFTEEN_ARTIFACT,
    FIFTEEN_ARTIFACT_GATE_INTAKE,
    FIFTEEN_ARTIFACT_ALLIANCE_FRIENDLY,
    TWELVE_ARTIFACT,
    TWELVE_ARTIFACT_ALLIANCE_FRIENDLY,
    FAR_ZONE_6_ARTIFACT_FROM_HP,
    FAR_ZONE_15_ARTIFACT_FROM_HP_SM,
    FAR_ZONE_9_ARTIFACT_FROM_HP
}

@Autonomous(name = "Auto V3", group = "1", preselectTeleOp = "0: Main Teleop")
public class AutoV3 extends LinearOpMode {
    private boolean useDefault = true;
    private boolean robotStartIsSet = false;
    private V3AutoDefault defaultAuto = V3AutoDefault.EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY;
    private RobotManager robot;
    private RunSide runSide = RunSide.CLOSE_ZONE;
    private AutoProgram internalProgram;
    private ArrayList<AutoCommandRepository.AutoCommand> commands = new ArrayList<>();

    private void populateCommandsFromDefault() {
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

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
                break;
            case F1B_EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                double offset = .3;

                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true, offset));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false, offset));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false, offset));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine(false));
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
                break;
            case TWENTY_ONE_ARTIFACT_ALLIANCE_FRIENDLY:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeMidLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeGate(false));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeCloseLine(false));
                commands.add(new AutoCommandRepository.ScoreArtifactsAndPark());

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
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

                runSide = RunSide.CLOSE_ZONE;
                break;
            case FAR_ZONE_6_ARTIFACT_FROM_HP:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));

                runSide = RunSide.FAR_ZONE;
                break;
            case FAR_ZONE_15_ARTIFACT_FROM_HP_SM:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeFarLine());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.Park());

                runSide = RunSide.FAR_ZONE;
                break;
            case FAR_ZONE_9_ARTIFACT_FROM_HP:
                commands.add(new AutoCommandRepository.ScoreArtifacts(true));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer());
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));
                commands.add(new AutoCommandRepository.IntakeHumanPlayer(true));
                commands.add(new AutoCommandRepository.ScoreArtifacts(false));

                runSide = RunSide.FAR_ZONE;
                break;
        }
    }

    private String getDefaultUserLabel(V3AutoDefault ofType) {
        switch (ofType) {
            case EIGHTEEN_ARTIFACT:
                return "Close Zone: 18 Artifact Auto";
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                return "Close Zone: 18 Artifact Alliance Friendly Auto";
            case F1B_EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY:
                return "(Field 1 Blue) Close Zone: 18 Artifact Alliance Friendly Auto";
            case EIGHTEEN_ARTIFACT_ALLIANCE_FRIENDLY_EXTRA_PUSH:
                return "Close Zone: 18 Artifact Alliance Friendly Auto with an Extra Gate Push";
            case TWENTY_ONE_ARTIFACT_ALLIANCE_FRIENDLY:
                return "Close Zone: 21 Artifact Alliance Friendly Auto";
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
            case FAR_ZONE_15_ARTIFACT_FROM_HP_SM:
                return "Far Zone: 15 Artifact (from far spike mark and human player station)";
            case FAR_ZONE_6_ARTIFACT_FROM_HP:
                return "Far Zone: 6 Artifact (all from human player station)";
        }

        return "Unimplemented Default Auto";
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotManager(this);

        robot.setShooterAimPolicy(ShooterAimPolicy.TURRET);
        robot.setShooterControlPolicy(ShooterControlPolicy.MANUAL);
        robot.enableAutoTransferStop();
        robot.disableDebugPrinting();
        robot.enableVelocityCompensation();
        robot.setShootingStyle(ShootingStyle.LARGE_ARC);
        robot.disableHoodCompensation();
        robot.disableOnlyShootInZone();
        robot.disableWaitForVelocityToShoot();
        robot.disablePoweredHold();

        robot.initialise();
        robot.recalibrateIMU();

        int defaultSelection = defaultAuto.ordinal();
        boolean defaultUpdated = true;

        robot.enableTurretRelativeControl();
        robot.enableTurret();
        robot.setConstantTurretHeadingGoal(180, AngleUnit.DEGREES);

        Parameters.SHOOTER_GOAL_CLOSE_BLUE_AIM = new Pose(-105, 10);
        Parameters.SHOOTER_GOAL_CLOSE_RED_AIM = new Pose(0, 0);

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

                populateCommandsFromDefault();

                defaultUpdated = false;
                useDefault = true;
            }

            telemetry.addData("Alliance Side: ", robot.getAllianceSide() == AllianceSides.BLUE ? "Blue" : "Red");
            telemetry.addData("Autonomous Default: ", getDefaultUserLabel(defaultAuto));
            telemetry.addData("Robot Heading: ", robot.getPose().getHeading());
            telemetry.addLine();
            telemetry.addLine("Auto Program:");
            telemetry.addData("Starting Side: ", runSide);

            for(AutoCommandRepository.AutoCommand command : commands) {
                telemetry.addData("Command", command.getUserLabel());
            }

            updateRobotStart();

            robot.update();
            telemetry.update();
        }

        waitForStart();
        updateRobotStart();

        robot.disableTurretRelativeControl();
        robot.startAimingAtGoal();
        robot.setTurretBacklashPolicy(TurretBacklashPolicy.MITIGATE_ALWAYS);

        internalProgram = new AutoProgram(robot);

        if (useDefault)
            populateCommandsFromDefault();

        internalProgram.setCommands(commands);
        internalProgram.execute(runSide, getStartPose());

        robot.safeSleep(1000);
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

        if (runSide == RunSide.CLOSE_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_CLOSE_START : Parameters.BLUE_CLOSE_START;
        } else if (runSide == RunSide.FAR_ZONE) {
            startPose = robot.getAllianceSide() == AllianceSides.RED ? Parameters.RED_FAR_START : Parameters.BLUE_FAR_START;
        }

        return startPose;
    }
}
