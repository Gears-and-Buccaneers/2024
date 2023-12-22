package frc.lib.hardware.sensor.imu;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon implements IMU {
  private Pigeon2 pigeon2;

  public Pigeon(int canID) {
    pigeon2 = new Pigeon2(canID);
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
}
