package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.PIDFController;

public class ComplexMotor {
    private LinearOpMode opMode;
    private DcMotorEx thisMotor = null;
    private ComplexMotorModes currentMode = ComplexMotorModes.RAW_POWER;
    private double motorPower = 0;
    private double targetVelo = 0;
    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);
    private boolean useCustomVelo = true;
    private ComplexMotor childMotor = null;
    protected boolean isThisChild = false;

    public ComplexMotor(String hwName, LinearOpMode newOpMode) {
        opMode = newOpMode;
        this.thisMotor = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.thisMotor.setMotorEnable();
    }

    public void addChildMotor(ComplexMotor newChildMotor) {
        this.childMotor = newChildMotor;
        this.childMotor.setMode(ComplexMotorModes.RAW_POWER);
        this.childMotor.setEncoderState(false);
        this.childMotor.isThisChild = true;
        isThisChild = false;

        if (thisMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            this.childMotor.enableBrake();
        } else {
            this.childMotor.enableFloat();
        }
    }

    public DcMotorEx getMotor() {
        return thisMotor;
    }

    public void enableBrake() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (this.childMotor != null) {
            this.childMotor.enableBrake();
        }
    }

    public void enableFloat() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (this.childMotor != null) {
            this.childMotor.enableFloat();
        }
    }

    public void setEncoderState(boolean use) {
        this.thisMotor.setMode(use ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVelocity() {
        return thisMotor.getVelocity(AngleUnit.DEGREES);
    }

    public void useCustomVeloPIDLoop(boolean newUseCustomVelo) {
        useCustomVelo = newUseCustomVelo;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        velocityController.setPIDF(p, i, d, f);
    }

    public void setMode(ComplexMotorModes newMode) {
        currentMode = newMode;
    }

    public void setVelocity(double newVelo) {
        targetVelo = newVelo;
    }

    public void resetEncoder() {
        DcMotor.RunMode oldState = this.thisMotor.getMode();

        this.thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.thisMotor.setMode(oldState);
    }

    public void setReversed(boolean reversed) {
        thisMotor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (isThisChild) return;

        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            if (useCustomVelo) {
                velocityController.setSetPoint(targetVelo);
                motorPower = velocityController.calculate(thisMotor.getVelocity(AngleUnit.DEGREES));

                if (targetVelo == 0) motorPower = 0; // simple override

                opMode.telemetry.addData("pid motor power: ", motorPower);

                thisMotor.setPower(motorPower);
                if (childMotor != null) childMotor.setPower(motorPower);
            } else {
                thisMotor.setVelocity(targetVelo, AngleUnit.DEGREES);
            }
        } else if (currentMode == ComplexMotorModes.RAW_POWER) {
            thisMotor.setPower(motorPower);
        }

    }

    public double getCurrent() {
        return thisMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void setPower(double power) {
        if (currentMode == ComplexMotorModes.RAW_POWER) {
            motorPower = power;
        }
    }

    public boolean atVelocity() {
        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            if (useCustomVelo) {
                return velocityController.atSetPoint();
            } else {
                return abs(thisMotor.getVelocity(AngleUnit.DEGREES) - targetVelo) < 3;
            }
        }
        return true;
    }
}
