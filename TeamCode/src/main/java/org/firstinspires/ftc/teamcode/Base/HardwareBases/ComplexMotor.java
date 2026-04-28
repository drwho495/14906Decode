package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.HardwareUtils;
import org.firstinspires.ftc.teamcode.Base.Helpers.PIDFController;
import org.firstinspires.ftc.teamcode.Base.Parameters;

import java.util.concurrent.TimeUnit;

public class ComplexMotor {
    private LinearOpMode opMode;
    private DcMotorEx thisMotor = null;
    private ComplexMotorModes currentMode = ComplexMotorModes.RAW_POWER;
    private double motorPower = 0;
    private double targetVelocity = 0;
    private double currentVelocity;
    private double voltageTarget = 12.0;
    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);
    private boolean useCustomVelocity = true;
    private VoltageSensor vSensor = null;
    private ComplexMotor childMotor = null;
    private double currentLimit = 0;
    private CurrentUnit currentLimitUnit = CurrentUnit.AMPS;

    public ComplexMotor(String hwName, LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        this.thisMotor = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.thisMotor.setMotorEnable();
    }

    public ComplexMotor(String hwName, LinearOpMode newOpMode, VoltageSensor vSensor) {
        this.opMode = newOpMode;
        this.vSensor = vSensor;
        this.thisMotor = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.thisMotor.setMotorEnable();
    }

    public void enableBrake() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableFloat() {
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setLinkedMotor(ComplexMotor linkedMotor) {
        this.childMotor = linkedMotor;
        this.childMotor.setEncoderState(true);
        this.childMotor.setMode(ComplexMotorModes.RAW_POWER);
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

    public boolean useCustomVeloPIDLoop() {
        return useCustomVelocity;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (useCustomVelocity) {
            velocityController.setPIDF(p, i, d, f);
        } else {
            thisMotor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    public double[] getVelocityPIDFCoefficients() {
        if (useCustomVelocity) {
            return velocityController.getCoefficients();
        } else {
            PIDFCoefficients coefficients = thisMotor.getPIDFCoefficients(thisMotor.getMode());

            return new double[]{coefficients.p, coefficients.i, coefficients.d, coefficients.f};
        }
    }

    public void setMode(ComplexMotorModes newMode) {
        currentMode = newMode;
    }
    public ComplexMotorModes getMode() {
        return currentMode;
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
            if (useCustomVelocity) {
                if (targetVelocity != 0) {
                    velocityController.setSetPoint(targetVelocity);
                    motorPower = velocityController.calculate(currentVelocity);

                    if (vSensor != null) {
                        motorPower *= (Parameters.SHOOTER_VOLTAGE_TARGET / vSensor.getVoltage());
                    }

                    HardwareUtils.optimizeMethod(motorPower, thisMotor, thisMotor::setPower);

                    if (childMotor != null) {
                        HardwareUtils.optimizeMethod(motorPower, childMotor, childMotor::setPower);
                    }
                } else {
                    HardwareUtils.optimizeMethod(0, thisMotor, thisMotor::setPower);

                    if (childMotor != null) {
                        HardwareUtils.optimizeMethod(0, childMotor, childMotor::setPower);
                    }
                }
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

            if (childMotor != null) {
                childMotor.setPower(motorPower);
            }
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

    public boolean atVelocity(double error) {
        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            return abs(currentVelocity - targetVelocity) <= error;
        }
        return true;
    }

    public void setCurrentLimit(CurrentUnit unit, double value) {
        if (unit != currentLimitUnit || value != currentLimit)
            thisMotor.setCurrentAlert(value, unit);
    }

    public boolean isOverCurrent(CurrentUnit unit, double value) {
        setCurrentLimit(unit, value);

        return isOverCurrent();
    }

    public boolean isOverCurrent() {
        return thisMotor.isOverCurrent();
    }
}
