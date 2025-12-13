package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.HardwareUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.PIDFController;

public class ComplexMotor {
    private LinearOpMode opMode;
    private DcMotorEx thisMotor = null;
    private ComplexMotorModes currentMode = ComplexMotorModes.RAW_POWER;
    private double motorPower = 0;
    private double targetVelocity = 0;
    private double currentVelocity;
    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);
    private boolean useCustomVelocity = true;

    public ComplexMotor(String hwName, LinearOpMode newOpMode) {
        opMode = newOpMode;
        this.thisMotor = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.thisMotor.setMotorEnable();
    }

    public void enableBrake() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableFloat() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setEncoderState(boolean use) {
        this.thisMotor.setMode(use ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public void useCustomVeloPIDLoop(boolean newUseCustomVelo) {
        useCustomVelocity = newUseCustomVelo;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (useCustomVelocity) {
            velocityController.setPIDF(p, i, d, f);
        } else {
            thisMotor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    public void setMode(ComplexMotorModes newMode) {
        currentMode = newMode;
    }

    public void setVelocity(double newVelo) {
        targetVelocity = newVelo;
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
        currentVelocity = thisMotor.getVelocity(AngleUnit.DEGREES);

        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            opMode.telemetry.addData("current velo: ", currentVelocity);

            if (useCustomVelocity) {
                velocityController.setSetPoint(targetVelocity);
                motorPower = velocityController.calculate(currentVelocity);

                HardwareUtils.optimizeMethod(motorPower, thisMotor, thisMotor::setPower);
            } else {
                if (targetVelocity != 0) {
                    double prevValue = HardwareUtils.previousValues.getOrDefault(thisMotor, Double.NaN);

                    if (Double.isNaN(prevValue) || targetVelocity != prevValue) {
                        thisMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
                        HardwareUtils.previousValues.put(thisMotor, targetVelocity);
                    }
                } else {
                    HardwareUtils.optimizeMethod(0, thisMotor, thisMotor::setPower);
                }
            }
        } else if (currentMode == ComplexMotorModes.RAW_POWER) {
            HardwareUtils.optimizeMethod(motorPower, thisMotor, thisMotor::setPower);
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
            if (useCustomVelocity) {
                return velocityController.atSetPoint();
            } else {
                return abs(currentVelocity - targetVelocity) < 5;
            }
        }
        return true;
    }
}
