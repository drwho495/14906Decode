package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.PIDFController;
import org.firstinspires.ftc.teamcode.Base.Parameters;

public class ComplexMotor {
    private LinearOpMode opMode;
    private DcMotorEx motorInterface = null;
    private ComplexMotorModes currentMode = ComplexMotorModes.RAW_POWER;
    private ComplexMotor encoderMotor = null;
    private double motorPower = 0;
    private double lastMotorPower = 0;
    private double lastMotorVelocity = 0;
    private double motorVelocity = 0;
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
        this.motorInterface = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.encoderMotor = this;
        this.motorInterface.setMotorEnable();
    }

    public ComplexMotor(String hwName, LinearOpMode newOpMode, VoltageSensor vSensor) {
        this.opMode = newOpMode;
        this.vSensor = vSensor;
        this.motorInterface = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);
        this.encoderMotor = this;
        this.motorInterface.setMotorEnable();
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        if (obj instanceof ComplexMotor) {
            ComplexMotor objComplexMotor = (ComplexMotor) obj;

            return objComplexMotor.motorInterface == motorInterface;
        }
        return false;
    }

    public void enableBrake() {
        this.motorInterface.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableFloat() {
        this.motorInterface.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setLinkedMotor(ComplexMotor linkedMotor) {
        this.childMotor = linkedMotor;
        this.childMotor.setMode(ComplexMotorModes.RAW_POWER);
    }

    public void setEncoderMotor(ComplexMotor motor) {
        encoderMotor = motor;
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
            motorInterface.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    public double[] getVelocityPIDFCoefficients() {
        if (useCustomVelocity) {
            return velocityController.getCoefficients();
        } else {
            PIDFCoefficients coefficients = motorInterface.getPIDFCoefficients(motorInterface.getMode());

            return new double[]{coefficients.p, coefficients.i, coefficients.d, coefficients.f};
        }
    }

    public void setMotorRunMode(DcMotor.RunMode runMode) {
        motorInterface.setMode(runMode);
    }

    public void setMode(ComplexMotorModes newMode) {
        currentMode = newMode;
    }

    public ComplexMotorModes getMode() {
        return currentMode;
    }

    public void setVelocity(double newVelo) {
        motorVelocity = newVelo;
    }

    public void resetEncoder() {
        DcMotor.RunMode oldState = this.encoderMotor.motorInterface.getMode();

        this.encoderMotor.motorInterface.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoderMotor.motorInterface.setMode(oldState);
    }

    public void setReversed(boolean reversed) {
        motorInterface.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (encoderMotor != null) {
            currentVelocity = encoderMotor.motorInterface.getVelocity(AngleUnit.DEGREES);
        } else {
            currentVelocity = 0;
        }

        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            if (useCustomVelocity) {
                if (motorVelocity != 0) {
                    velocityController.setSetPoint(motorVelocity);
                    motorPower = velocityController.calculate(currentVelocity);

                    if (vSensor != null) {
                        motorPower *= (Parameters.SHOOTER_VOLTAGE_TARGET / vSensor.getVoltage());
                    }

                    if (lastMotorPower != motorPower) {
                        motorInterface.setPower(motorPower);

                        if (childMotor != null) {
                            childMotor.setPower(motorPower);
                        }
                    }
                } else {
                    if (lastMotorPower != 0) {
                        motorInterface.setPower(0);

                        if (childMotor != null) {
                            childMotor.setPower(0);
                        }
                    }
                }
            } else {
                if (motorVelocity != 0) {
                    if (lastMotorVelocity != motorVelocity) {
                        motorInterface.setVelocity(motorVelocity, AngleUnit.DEGREES);
                    }
                } else {
                    if (lastMotorPower != 0) {
                        motorInterface.setPower(0);
                    }
                }
            }
        } else if (currentMode == ComplexMotorModes.RAW_POWER) {
            if (lastMotorPower != motorPower) {
                motorInterface.setPower(motorPower);

                if (childMotor != null) {
                    childMotor.setPower(motorPower);
                }
            }
        }

        lastMotorPower = motorPower;
        lastMotorVelocity = motorVelocity;
    }

    public double getCurrent() {
        return motorInterface.getCurrent(CurrentUnit.AMPS);
    }

    public void setPower(double power) {
        if (currentMode == ComplexMotorModes.RAW_POWER) {
            motorPower = power;
        }
    }

    public boolean atVelocity(double error) {
        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            return abs(currentVelocity - motorVelocity) <= error;
        }
        return true;
    }

    public void setCurrentLimit(CurrentUnit unit, double value) {
        if (unit != currentLimitUnit || value != currentLimit)
            motorInterface.setCurrentAlert(value, unit);
    }

    public boolean isOverCurrent(CurrentUnit unit, double value) {
        setCurrentLimit(unit, value);

        return isOverCurrent();
    }

    public boolean isOverCurrent() {
        return motorInterface.isOverCurrent();
    }
}
