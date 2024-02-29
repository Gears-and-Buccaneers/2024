package frc.system.mechanism.components;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.hardware.ProfiledMotor;

public class Shooter {
	private final ProfiledMotor motor;

	private final DoubleEntry ntVelocity;
	private final DoubleEntry ntDeadband;

	private double lastSpeed;

	public Shooter(ProfiledMotor motor, double defaultVelocity, double defaultDeadband) {
		this.motor = motor;

		NetworkTableInstance nt = NetworkTableInstance.getDefault();

		ntVelocity = nt.getDoubleTopic("Mechanism/Shooter/Velocity").getEntry(defaultVelocity);
		ntDeadband = nt.getDoubleTopic("Mechanism/Shooter/Deadband").getEntry(defaultDeadband);
	}

	public Command run() {
		return new Command() {
			@Override
			public void initialize() {
				motor.setVelocity(ntVelocity.get());
			}

			@Override
			public void end(boolean interrupted) {
				motor.setPercent(0);
			}
		};
	}

	Command reverse() {
		return new Command() {
			@Override
			public void initialize() {
				motor.setVelocity(-ntVelocity.get());
			}

			@Override
			public void end(boolean interrupted) {
				motor.setPercent(0);
			}
		};
	}

	public Command waitPrimed() {
		return new WaitUntilCommand(() -> {
			if (lastSpeed == 0)
				return false;

			return Math.abs(motor.velocity() - lastSpeed) <= ntDeadband.get();
		});
	}
}
