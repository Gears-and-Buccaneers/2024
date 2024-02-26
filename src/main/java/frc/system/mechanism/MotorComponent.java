package frc.system.mechanism;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.Motor;

/**
 * A class which includes a motor and a NetworkTableEntry which supplies its
 * percent-output speed.
 */
public abstract class MotorComponent {
	private final Motor motor;
	private final DoubleEntry ntSpeed;

	protected MotorComponent(Motor motor, String name, double defaultSpeed) {
		this.motor = motor;
		this.ntSpeed = NetworkTableInstance.getDefault().getDoubleTopic(name).getEntry(defaultSpeed);
	}

	public Command run() {
		return new Command() {
			@Override
			public void initialize() {
				motor.setPercent(ntSpeed.get());
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
				motor.setPercent(-ntSpeed.get());
			}

			@Override
			public void end(boolean interrupted) {
				motor.setPercent(0);
			}
		};
	}
}
