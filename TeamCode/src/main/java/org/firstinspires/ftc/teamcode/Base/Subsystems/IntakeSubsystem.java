package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexMotorModes;
import org.firstinspires.ftc.teamcode.Base.Parameters;

import java.util.concurrent.TimeUnit;

public class IntakeSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
    private ComplexMotor intakeMotor1;
    private ComplexMotor intakeMotor2;
    private double intakePower = 0;
    private double intakeMotor1Limit = 1;
    private double intakeMotor2Limit = 1;
    private boolean autoDisableTransfer = true;
    private boolean transferDisabled = false;
    private ElapsedTime transferDisableTimeout = new ElapsedTime();
    private boolean transferDisabling = false;

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
        intakeMotor2.setEncoderState(false);
        intakeMotor2.setReversed(true);
        intakeMotor2.enableBrake();
        intakeMotor2.setPower(0);
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

    public double getTransferLockTime() {
        return (transferDisabling && autoDisableTransfer) ? transferDisableTimeout.time(TimeUnit.MILLISECONDS) : -1;
    }

    public void powerIntakeIdle() {
        intakePower = Parameters.INTAKE_IDLE;
    }

    public void powerIntakeOff() {
        intakePower = 0;
    }

    public boolean isAutoTransferOffEnabled() {
        return autoDisableTransfer;
    }

    public void setPowerLimits(double motor1Limit, double motor2Limit) {
        intakeMotor1Limit = motor1Limit;
        intakeMotor2Limit = motor2Limit;
    }

    public void enableAutoDisableTransfer() {
        if (!autoDisableTransfer) {
            autoDisableTransfer = true;
            transferDisabled = false;
            transferDisabling = false;
        }
    }

    public void disableAutoDisableTransfer() {
        if (autoDisableTransfer) {
            autoDisableTransfer = false;
            transferDisabled = false;
            transferDisabling = false;
        }
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        intakeMotor1.setPower(Range.clip(intakePower, -1, intakeMotor1Limit));
        intakeMotor1.update();

        if (transferDisabled && autoDisableTransfer) {
            intakeMotor2.setPower(0);
        } else {
            intakeMotor2.setPower(Range.clip(intakePower, -1, intakeMotor2Limit));
        }

        intakeMotor2.update();

        double intakeMotor2Current = intakeMotor2.getCurrent();

        if (intakeMotor2Current > 2 && autoDisableTransfer) {
            if (!transferDisabling) {
                transferDisabling = true;
                transferDisableTimeout.reset();
            }

            if (transferDisableTimeout.time(TimeUnit.MILLISECONDS) > 750) {
                transferDisabled = true;
            }
        } else {
            transferDisabling = false;
        }
    }

    public void setIntakePower(double newPower) {
        intakePower = newPower;
    }

    public boolean isTransferStalled() {
        return transferDisabled;
    }
}
