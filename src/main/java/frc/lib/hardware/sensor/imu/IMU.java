package frc.lib.hardware.sensor.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.hardware.HardwareRequirements;
import org.littletonrobotics.junction.LogTable;

public interface IMU extends HardwareRequirements {

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

  // Loging (Shold be default can always overdie)
  @Override
  default void toLog(LogTable table) {
    table.put(this.getClass().getSimpleName() + "/Pitch", getPitch().getDegrees());
    table.put(this.getClass().getSimpleName() + "/PitchVelocity", getPitchVelocity());
    table.put(this.getClass().getSimpleName() + "/Yaw", getYaw().getDegrees());
    table.put(this.getClass().getSimpleName() + "/YawVelocity", getYawVelocity());
    table.put(this.getClass().getSimpleName() + "/Roll", getRoll().getDegrees());
    table.put(this.getClass().getSimpleName() + "/RollVelocity", getRollVelocity());
  }

  @Override
  default void fromLog(LogTable table) {
  }
}
