package frc.hardware.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.hardware.Motor;

public class SRX implements Motor {
    final TalonSRX core;
    boolean invert = false;

    public SRX(int id) {
        core = new TalonSRX(id);
    }

    public SRX invert() {
        invert = true;
        return this;
    }

    @Override
    public void setPercent(double percent) {
        core.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO
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
