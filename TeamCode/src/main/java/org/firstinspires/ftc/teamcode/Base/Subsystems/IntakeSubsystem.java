package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ColorRangefinder;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.Parameters;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor intakeMotor1;
    private ComplexMotor intakeMotor2;
    private double intakeMotor1Power = 0;
    private double intakeMotor2Power = 0;
    private double intakeMotor1Limit = 1;
    private double intakeMotor2Limit = 1;

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
        intakeMotor1 = new ComplexMotor("intakeMotor1", thisOpMode);
        intakeMotor1.setMode(ComplexMotorModes.RAW_POWER);
        intakeMotor1.setEncoderState(false);
        intakeMotor1.enableBrake();
        intakeMotor1.setPower(0);

        intakeMotor2 = new ComplexMotor("intakeMotor2", thisOpMode);
        intakeMotor2.setMode(ComplexMotorModes.RAW_POWER);
        intakeMotor2.setEncoderState(true);
        intakeMotor2.setReversed(true);
        intakeMotor2.enableBrake();
        intakeMotor2.setPower(0);
    }

    public double getIntakePower() {
        return (intakeMotor1Power + intakeMotor2Power) / 2;
    }

    public boolean isIntakeOn() {
        return (intakeMotor1Power != 0 && intakeMotor2Power != 0);
    }

    public void powerIntakeOff() {
        intakeMotor1Power = 0;
        intakeMotor2Power = 0;
    }

    public void setPowerLimits(double motor1Limit, double motor2Limit) {
        intakeMotor1Limit = motor1Limit;
        intakeMotor2Limit = motor2Limit;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        intakeMotor1.setPower(Range.clip(intakeMotor1Power, -1, intakeMotor1Limit));
        intakeMotor2.setPower(Range.clip(intakeMotor2Power, -1, intakeMotor2Limit));
        intakeMotor1.update();

        intakeMotor2.update();
    }

    public void setIntakePower(double newPower) {
        intakeMotor1Power = newPower;
        intakeMotor2Power = newPower;
    }

    public void setIntakePowers(double motor1Power, double motor2Power) {
        intakeMotor1Power = motor1Power;
        intakeMotor2Power = motor2Power;
    }

    public boolean isTransferOverCurrent(CurrentUnit unit, double value) {
        return intakeMotor2.isOverCurrent(unit, value);
    }
}
