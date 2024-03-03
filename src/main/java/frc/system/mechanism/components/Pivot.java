package frc.system.mechanism.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.ProfiledMotor;

public class Pivot {
	final double o;
	final double a;

	final Command gotoIntake;
	final Command gotoAmp;

	public final ProfiledMotor motor;
	final DoubleEntry ntDeadband;

	/**
	 * Assumes 90 degree angle is 0 and the intake position has negative rotational
	 * sign.
	 */
	public Pivot(ProfiledMotor motor, double defaultDeadband, Rotation2d intakePosition, Rotation2d ampPosition,
			double armLength,
			Rotation2d armOffset) {
		this.motor = motor;
		ntDeadband = NetworkTableInstance.getDefault().getDoubleTopic("Mechanism/Pivot/Deadband")
				.getEntry(defaultDeadband);

		o = armOffset.getRadians();
		a = armLength * Math.sin(armOffset.getRadians());
		gotoIntake = goTo(intakePosition.getRotations());
		gotoAmp = goTo(intakePosition.getRotations());
	}

	public Command aim(double pitchRad, double dist) {
		double rotations = Units.radiansToRotations(Math.PI - o - Math.asin(a / dist) + pitchRad);
		return goTo(rotations);
	}

	public Command toAmp() {
		return gotoAmp;
	}

	public Command toIntake() {
		return gotoIntake;
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
