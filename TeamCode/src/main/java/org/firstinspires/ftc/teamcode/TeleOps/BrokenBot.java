package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.bedroBathing.tuning.FollowerConstants;

@TeleOp(name = "Broken Bot", group = "1")
public class BrokenBot extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                leftFront.setPower(1);
            } else {
                leftFront.setPower(0);
            }

            if (gamepad1.dpad_down) {
                rightFront.setPower(1);
            } else {
                rightFront.setPower(0);
            }

            if (gamepad1.dpad_left) {
                leftRear.setPower(1);
            } else {
                leftRear.setPower(0);
            }

            if (gamepad1.dpad_right) {
                rightRear.setPower(1);
            } else {
                rightRear.setPower(0);
            }

            telemetry.addData("rf: ", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lf: ", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rr: ", rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lr: ", leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
