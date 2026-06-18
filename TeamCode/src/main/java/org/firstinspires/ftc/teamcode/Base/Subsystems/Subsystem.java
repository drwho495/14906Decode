package org.firstinspires.ftc.teamcode.Base.Subsystems;

// subsystems no longer take states, only the manager uses those.
// i may make it take options eventually

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.HardwareTable;

public abstract class Subsystem {
    public abstract void setLinearTeleop(LinearOpMode opMode);
    public abstract void update();
    public abstract void initialiseHardware(HardwareTable hardwareTable);
}
