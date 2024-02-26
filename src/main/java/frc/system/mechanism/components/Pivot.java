package frc.system.mechanism.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.ProfiledMotor;

public class Pivot {
	final ProfiledMotor motor;
	final DigitalInput limit;

	public Pivot(ProfiledMotor motor, DigitalInput limit) {
		this.motor = motor;
		this.limit = limit;
	}

	public Command aim(Rotation2d angle) {
		double rotations = angle.getRotations();

		return new Command() {
			@Override
			public void initialize() {
				// TODO Auto-generated method stub
				super.initialize();
			}

			@Override
			public boolean isFinished() {
				return Math.abs(motor.velocity()) <= 0.1 && Math.abs(motor.position() - rotations) <= 0.1;
			}
		};
	}

	public Command toIntake() {
		return null;
	}
}
