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
}
