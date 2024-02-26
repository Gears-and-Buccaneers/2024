package frc.hardware.profiledmotor;

import frc.hardware.ProfiledMotor;

public class Simulated implements ProfiledMotor {
	double position = 0;
	double velocity = 0;

	@Override
	public double position() {
		return position;
	}

	@Override
	public double velocity() {
		return velocity;
	}

	@Override
	public void setPosition(double position) {
		System.out.println("Set position to " + position);
		this.position = position;
	}

	@Override
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	@Override
	public void setPercent(double percent) {
	}

	@Override
	public void setVoltage(double voltage) {
	}
}
