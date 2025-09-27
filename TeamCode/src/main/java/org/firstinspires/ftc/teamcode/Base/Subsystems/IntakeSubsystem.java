package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.HardwareBases.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.Parameters;

public class IntakeSubsystem extends Subsystem {
    private LinearOpMode thisOpMode = null;
//    private double intakePower = 0;
    private boolean clawClosed = false;
    private double gateServoPos = Parameters.GATE_OPEN_POSITION;
    private ComplexServo clawServo;
    private ComplexServo wristServo;
    private ComplexServo gateServo;
    private double wristPosition = Parameters.WRIST_DOWN;

    @Override
    public void setLinearTeleop(LinearOpMode newOpMode) {
        thisOpMode = newOpMode;
    }

    /**
     * this method transfers the ball into the shooter, which should be running.
     * doing this should immediately launch the ball.
     */
//    public void transferBall() {
//
//    }

//    public void setIntakePower(double newPower) {
//        intakePower = newPower;
//    }

    public boolean isClawClosed() {
        return clawClosed;
    }

    public void clawGrab() {
        clawClosed = true;
    }

    public void clawRelease() {
        clawClosed = false;
    }

    public void clawToggle() {
        clawClosed = !clawClosed;
    }

    /**
     * this method controls if the claw is open or not
     * true for closed
     * false for open
     */
    private void setClawState(boolean isClawClosed) {
        clawClosed = isClawClosed;
    }

    public void setWristPosition(double newPosition) {
        wristPosition = newPosition;
    }

    public void wristDown() {
        wristPosition = Parameters.WRIST_DOWN;
    }

    public void wristUp() {
        wristPosition = Parameters.WRIST_UP;
    }

    public void wristHold() {
        wristPosition = Parameters.WRIST_MID_POS;
    }

    @Override
    public void initialiseHardware() {
        clawServo = new ComplexServo(thisOpMode.hardwareMap, "clawServo", 0, 180);

        wristServo = new ComplexServo(thisOpMode.hardwareMap, "wristServo", 0, 180);
        wristServo.setInverted(true);

        gateServo = new ComplexServo(thisOpMode.hardwareMap, "gateServo", 0, 180);
    }

    public void openGate() {
        gateServoPos = Parameters.GATE_OPEN_POSITION;
    }

    public void closeGate() {
        gateServoPos = Parameters.GATE_CLOSED_POSITION;
    }

    public double getGatePosition() {
        return gateServoPos;
    }

    public boolean isGateClosed() {
        return gateServoPos == Parameters.GATE_CLOSED_POSITION;
    }

    @Override
    public void update() {
        if (!thisOpMode.opModeIsActive() || thisOpMode.isStopRequested()) return;

        if (clawClosed) {
            clawServo.turnToAngle(Parameters.CLAW_GRAB);
        } else {
            clawServo.turnToAngle(Parameters.CLAW_RELEASE);
        }

        gateServo.turnToAngle(gateServoPos);
        wristServo.turnToAngle(wristPosition);
    }
}
