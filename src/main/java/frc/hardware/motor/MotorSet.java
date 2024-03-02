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
}
