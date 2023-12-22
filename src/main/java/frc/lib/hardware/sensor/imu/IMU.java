package frc.lib.hardware.sensor.imu;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The IMU interface defines methods for retrieving the pitch, yaw, and roll values of an IMU
 * (Inertial Measurement Unit) sensor. It also includes a default method for obtaining a Rotation3d
 * object using the pitch, yaw, and roll values.
 *
 * <p>Example Usage: IMU imu = new MyIMU(); // Instantiate an IMU object double pitch =
 * imu.getPitch(); // Get the pitch value double yaw = imu.getYaw(); // Get the yaw value double
 * roll = imu.getRoll(); // Get the roll value Rotation3d rotation = imu.getRotation3d(); // Get the
 * Rotation3d object
 */
public interface IMU {
  double getPitch();

  double getYaw();

  double getRoll();

  public default Rotation3d getRotation3d() {
    return new Rotation3d(getRoll(), getPitch(), getYaw());
  }
}
