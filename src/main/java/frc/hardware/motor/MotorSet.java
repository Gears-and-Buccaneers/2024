package frc.hardware.motor;

import frc.hardware.Motor;

public class MotorSet implements Motor {
    private final Motor[] motors;

    public MotorSet(Motor[] motors) {
        this.motors = motors;
    }

    @Override
    public void setPercent(double percent) {
        for (Motor motor : motors)
            motor.setPercent(percent);
    }

    @Override
    public void setVoltage(double voltage) {
        for (Motor motor : motors)
            motor.setVoltage(voltage);
    }

    @Override
    public int getDeviceID() {
        return 0;
    }

    @Override
    public String getControlMode() {
        throw new UnsupportedOperationException("Unimplemented method 'getControlMode'");
    }

    @Override
    public double getDeviceTemp() {
        throw new UnsupportedOperationException("Unimplemented method 'getDeviceTemp'");
    }

    @Override
    public double getSupplyCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
    }

    @Override
    public double getStatorCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getStatorCurrent'");
    }

    @Override
    public double getMotorVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getMotorVoltage'");
    }
}
