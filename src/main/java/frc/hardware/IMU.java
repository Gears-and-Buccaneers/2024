package frc.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMU {
	/** Gets the measured yaw of the IMU. */
	Rotation2d yaw();
}
