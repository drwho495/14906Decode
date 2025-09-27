package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Helpers.PIDFController;
import org.firstinspires.ftc.teamcode.bedroBathing.localization.Pose;
import org.firstinspires.ftc.teamcode.bedroBathing.pathGeneration.Vector;

public class ComplexMotor {
    private LinearOpMode opMode;
    private DcMotorEx thisMotor = null;
    private ComplexMotorModes currentMode = ComplexMotorModes.RAW_POWER;
    private double motorPower = 0;
    private double targetVelo = 0;
    private PIDFController velocityController = new PIDFController(0, 0, 0, 0);
    private boolean useCustomVelo = true;

    public ComplexMotor(String hwName, LinearOpMode newOpMode) {
        opMode = newOpMode;
        this.thisMotor = this.opMode.hardwareMap.get(DcMotorEx.class, hwName);

        this.thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.thisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.thisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.thisMotor.setMotorEnable();
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

    public void setVelo(double newVelo) {
        targetVelo = newVelo;
    }

    public void resetEncoder() {
        this.thisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.thisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setReversed(boolean reversed) {
        thisMotor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        if (currentMode == ComplexMotorModes.USE_VELOCITY_PID) {
            if (useCustomVelo) {
                velocityController.setSetPoint(targetVelo);
                motorPower = velocityController.calculate(thisMotor.getVelocity(AngleUnit.DEGREES));

                thisMotor.setPower(motorPower);
            } else {
                thisMotor.setVelocity(targetVelo, AngleUnit.DEGREES);
            }
        } else if (currentMode == ComplexMotorModes.RAW_POWER) {
            thisMotor.setPower(motorPower);
        }

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
