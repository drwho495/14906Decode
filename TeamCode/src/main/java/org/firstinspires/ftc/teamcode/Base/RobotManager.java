package org.firstinspires.ftc.teamcode.Base;

// this is the file that all teleops and autos
// the hardware is inited here

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;

public class RobotManager {
    // hardware is defined here

    private OpModeStates currentState = OpModeStates.IDLE;
    private OpModeOptions managerOptions = new OpModeOptions();

    // do NOT add a constructor to any of the subsystems!
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public RobotManager(LinearOpMode opMode) {
        shooterSubsystem.setLinearTeleop(opMode);
    }

    public void initialiseHardware() {
        // hardware is inited here

        // put this now initialised hardware into the subsystems
        shooterSubsystem.setHardware(); // unimplemeted

    }

    public void setState(OpModeStates newState) {
        currentState = newState;
    }

    public void setManagerOptions(OpModeOptions newOptions) {
        managerOptions = newOptions;
    }

    public void update() {
        switch (currentState) {
            case IDLE:

                break;

            case INTAKE_SCORE:

                break;

            case PARK:

                break;
        }
    }
}