package frc.lib.hardware.sensor.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.lib.hardware.sensor.SensorRequirments;

public interface IMU extends SensorRequirments {

  Rotation2d getPitch();

  double getPitchVelocity();

  Rotation2d getYaw();

  double getYawVelocity();

  Rotation2d getRoll();

  double getRollVelocity();

  void offsetIMU(Rotation2d offset);

  default void zeroIMU() {
    offsetIMU(new Rotation2d());
  }

  public default Rotation3d getRotation3d() {
    return new Rotation3d(getRoll().getDegrees(), getPitch().getDegrees(), getYaw().getDegrees());
  }

}
