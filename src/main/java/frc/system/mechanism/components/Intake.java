package frc.system.mechanism.components;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.Motor;

public class Intake {
	private final Motor motor;
	private final DoubleEntry ntSpeed;

	public Intake(Motor motor, double defaultSpeed) {
		this.motor = motor;
		this.ntSpeed = NetworkTableInstance.getDefault().getDoubleTopic("Mechanism/Intake/Output")
				.getEntry(defaultSpeed);
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
