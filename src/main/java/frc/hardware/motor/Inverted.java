package frc.hardware.motor;

import frc.hardware.Motor;

public class Inverted implements Motor {
	Motor inner;

	public Inverted(Motor motor) {
		inner = motor;
	}

	@Override
	public void setPercent(double percent) {
		inner.setPercent(-percent);
	}

	@Override
	public void setVoltage(double voltage) {
		inner.setVoltage(-voltage);
	}
}
