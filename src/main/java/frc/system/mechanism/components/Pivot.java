package frc.system.mechanism.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.ProfiledMotor;

public class Pivot {
	final ProfiledMotor motor;
	final DoubleEntry ntDeadband;

	public Pivot(ProfiledMotor motor, double defaultDeadband) {
		this.motor = motor;
		ntDeadband = NetworkTableInstance.getDefault().getDoubleTopic("Mechanism/Pivot/Deadband")
				.getEntry(defaultDeadband);
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
				return Math.abs(motor.position() - rotations) < ntDeadband.get();
			}
		};
	}

	public Command toIntake() {
		return null;
	}
}
