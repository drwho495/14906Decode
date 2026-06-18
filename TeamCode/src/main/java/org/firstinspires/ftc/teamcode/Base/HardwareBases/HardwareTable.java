package org.firstinspires.ftc.teamcode.Base.HardwareBases;

import java.util.ArrayList;

public class HardwareTable {
    private ArrayList<ComplexServo> registeredServos = new ArrayList<>();
    private ArrayList<ComplexMotor> registeredMotors = new ArrayList<>();

    public HardwareTable() {
    }

    public void registerHardwareInterface(ComplexMotor motor) {
        if (!registeredMotors.contains(motor)) {
            registeredMotors.add(motor);
        }
    }

    public void registerHardwareInterface(ComplexServo servo) {
        if (!registeredServos.contains(servo)) {
            registeredServos.add(servo);
        }
    }

    public ArrayList<ComplexServo> getServos() {
        return registeredServos;
    }

    public ArrayList<ComplexMotor> getMotors() {
        return registeredMotors;
    }
}
