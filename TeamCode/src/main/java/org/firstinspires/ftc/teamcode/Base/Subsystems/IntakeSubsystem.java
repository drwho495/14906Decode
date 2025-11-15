package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;

public class IntakeSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor intakeMotor;
    private double intakePower = 0;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    /**
     * this method transfers the ball into the shooter, which should be running.
     * doing this should immediately launch the ball.
     */
    @Override
    public void initialiseHardware() {
        intakeMotor = new ComplexMotor("intakeMotor", thisOpMode);
        intakeMotor.setMode(ComplexMotorModes.RAW_POWER);
        intakeMotor.setPower(0);
    }

    public void powerIntakeOn() {
        intakePower = Parameters.INTAKE_SPEED;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public boolean isIntakeOn() {
        return intakePower != 0;
    }

    public void powerIntakeIdle() {
        intakePower = Parameters.INTAKE_IDLE;
    }

    public void powerIntakeOff() {
        intakePower = 0;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        intakeMotor.setMode(ComplexMotorModes.RAW_POWER);
        intakeMotor.enableBrake();
        intakeMotor.setEncoderState(false);
        intakeMotor.setPower(intakePower);
        intakeMotor.update();
    }

    public void setIntakePower(double newPower) {
        intakePower = newPower;
    }
}
