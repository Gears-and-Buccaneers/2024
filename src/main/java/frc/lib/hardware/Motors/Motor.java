package frc.lib.hardware.Motors;

import frc.lib.hardware.Motors.MotorControlers.MotorController;

public class Motor {
    public enum MotorType {
        CIM,
        Falcon500,
        VP775
    }

    private MotorController mController;
    private MotorType motorType;

    public Motor(MotorController motorController, MotorType motor) {
        this.mController = motorController;
        this.motorType = motor;
    }

    public Motor inverted(boolean enable) {
        mController.setInverted(enable);
        return this;
    }
}
