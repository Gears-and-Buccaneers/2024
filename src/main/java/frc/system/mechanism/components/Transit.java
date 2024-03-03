package frc.system.mechanism.components;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.Motor;

public class Transit {
	private final LaserCan sensor;
	private final double threshold;

	private final Motor motor;
	private final DoubleEntry ntSpeed;

	/**
	 * @param distanceThreshold The LaserCAN distance threshold, in millimeters,
	 *                          after which a game piece is considered to be in the
	 *                          transit.
	 */
	public Transit(Motor motor, double defaultSpeed, int laserCanId, double distanceThreshold) {
		this.motor = motor;
		this.ntSpeed = NetworkTableInstance.getDefault().getDoubleTopic("Mechanism/Transit/Output")
				.getEntry(defaultSpeed);
		sensor = new LaserCan(laserCanId);
		this.threshold = distanceThreshold;
	}

	public boolean hasPiece() {
		Measurement measurement = sensor.getMeasurement();
		return measurement != null && measurement.distance_mm < threshold;
	}

	public Command intake() {
		return new Command() {
			@Override
			public void initialize() {
				motor.setPercent(ntSpeed.get());
			}

			@Override
			public boolean isFinished() {
				return hasPiece();
			}

			@Override
			public void end(boolean interrupted) {
				motor.setPercent(0);
			}
		};
	}

	public Command shoot() {
		return new Command() {
			@Override
			public void initialize() {
				motor.setPercent(ntSpeed.get());
			}

			@Override
			public boolean isFinished() {
				return !hasPiece();
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
