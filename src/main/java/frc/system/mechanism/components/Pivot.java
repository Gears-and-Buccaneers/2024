package frc.system.mechanism.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.ProfiledMotor;

public class Pivot {
	final double a;
	final double intakePosition;

	final ProfiledMotor motor;
	final DoubleEntry ntDeadband;

	public Pivot(ProfiledMotor motor, double defaultDeadband, Rotation2d intakePosition, double armLength,
			Rotation2d armOffset) {
		this.motor = motor;
		ntDeadband = NetworkTableInstance.getDefault().getDoubleTopic("Mechanism/Pivot/Deadband")
				.getEntry(defaultDeadband);

		a = armLength * Math.sin(armOffset.getRadians());
		this.intakePosition = intakePosition.getRotations();
	}

	public static double calculateOffset(double armLength, Rotation2d armOffset) {
		return Units.radiansToRotations(Math.PI - armOffset.getRadians());
	}

	public Command aim(double pitchRad, double dist) {
		double rotations = Units.radiansToRotations(pitchRad - Math.asin(a / dist));
		return goTo(rotations);
	}

	public Command toIntake() {
		return goTo(intakePosition);
	}

	public Command goTo(double rotations) {
		return new Command() {
			@Override
			public void initialize() {
				motor.setPosition(rotations);
			}

			@Override
			public boolean isFinished() {
				return Math.abs(motor.position() - rotations) < ntDeadband.get();
			}
		};
	}
}
