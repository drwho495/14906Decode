package org.firstinspires.ftc.teamcode.Base.Auto;


import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Base.RobotManager;

import java.util.ArrayList;

// This class stores a list of commands and executes them when needed.
public class AutoProgram {
    // This is a list of default initialized commands that can be used in an OpMode.
    private ArrayList<AutoCommandRepository.AutoCommand> commands = new ArrayList<>();
    private AutoStartSide startSide = AutoStartSide.CLOSE_ZONE;
    private final RobotManager robot;
    private Pose startPose = new Pose();
    private String autoName = "Unnamed Autonomous Program";

    public AutoProgram(RobotManager robot, String autoName) {
        this.robot = robot;
        this.autoName = autoName;
    }

    public AutoProgram(RobotManager robot) {
        this.robot = robot;
    }

    public void setName(String name) {
        autoName = name;
    }

    public String getName() {
        return autoName;
    }

    public ArrayList<AutoCommandRepository.AutoCommand> getCommands() {
        return commands;
    }

    public void setStartSide(AutoStartSide startSide) {
        this.startSide = startSide;
    }

    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    public void execute(AutoStartSide startSide, Pose startPose) {
        setStartSide(startSide);
        setStartPose(startPose);
        execute();
    }

    public void setCommands(ArrayList<AutoCommandRepository.AutoCommand> commands) {
        this.commands = commands;
    }

    public void execute() {
        AutoCommandRepository.AutoCommand currentCommand = null;
        AutoCommandRepository.AutoCommand lastCommand = null;
        AutoCommandRepository.AutoCommand nextCommand = null;

        for (int i = 0; i < commands.size(); i++) {
            currentCommand = commands.get(i);

            if (i > 0) {
                lastCommand = commands.get(i - 1);
            }

            if ((i + 1) < commands.size()) {
                nextCommand = commands.get(i + 1);
            } else {
                nextCommand = null;
            }

            currentCommand.execute(
                    robot,
                    startPose,
                    startSide,
                    lastCommand,
                    nextCommand
            );
        }
    }
}
