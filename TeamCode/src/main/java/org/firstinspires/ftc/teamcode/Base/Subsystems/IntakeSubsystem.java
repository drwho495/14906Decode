package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class IntakeSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private double intakePower = 0;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    // cannot be abstracted because of differing method arguments (?)
    public void setHardware() {

    }

    /**
     * this method transfers the ball into the shooter, which should be running.
     * doing this should immediately launch the ball.
     */
    public void transferBall() {

    }

    public void setIntakePower(double newPower) {
        intakePower = newPower;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

    }
}
