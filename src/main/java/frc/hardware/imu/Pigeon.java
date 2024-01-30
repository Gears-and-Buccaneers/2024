package frc.hardware.imu;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.CorePigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.hardware.IMU;

public class Pigeon implements IMU {
	final StatusSignal<Double> yaw;

	public Pigeon(int id) {
		CorePigeon2 imu = new CorePigeon2(id);
		yaw = imu.getYaw();
	}

	@Override
	public Rotation2d yaw() {
		return Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble());
	}
}
