package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ShooterSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    // cannot be abstracted because of differing method arguments (?)
    public void setHardware() {

    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

    }
}
