package org.firstinspires.ftc.teamcode.Base.Auto.Misc;

import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;

import java.util.ArrayList;
import java.util.function.Consumer;

public class AutoProgramProfile {
    private String profileName = "Unimplemented Profile";
    private Consumer<ArrayList<AutoCommandRepository.AutoCommand>> commandApplier = null;

    public String getName() {
        return profileName;
    }

    public void setName(String newName) {
        this.profileName = newName;
    }

    public void setCommandApplier(Consumer<ArrayList<AutoCommandRepository.AutoCommand>> commandApplier) {
        this.commandApplier = commandApplier;
    }

    public void applyChanges(ArrayList<AutoCommandRepository.AutoCommand> commands) {
        if (commandApplier != null) {
            commandApplier.accept(commands);
        }
    }
}
