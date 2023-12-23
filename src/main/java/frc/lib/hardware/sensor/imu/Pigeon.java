package frc.lib.hardware.sensor.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon implements IMU {
  private Pigeon2 pigeon2;

  public Pigeon(int canID) {
    pigeon2 = new Pigeon2(canID);

    pigeon2.setYaw(0.0);
  }

  public Pigeon() {
    this(0);
  }

  @Override
  public double getPitch() {
    return pigeon2.getPitch().getValueAsDouble();
  }

  @Override
  public double getYaw() {
    return pigeon2.getYaw().getValueAsDouble();
  }

  @Override
  public double getRoll() {
    return pigeon2.getRoll().getValueAsDouble();
  }

  @Override
  public void zeroIMU() {
    pigeon2.reset();
  }

  @Override
  public boolean connected() {
    return true;
  }

  @Override
  public Rotation2d getYaw2d() {
    return Rotation2d.fromDegrees(getYaw());
  }
}
