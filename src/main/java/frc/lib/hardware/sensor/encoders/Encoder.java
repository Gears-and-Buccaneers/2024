package frc.lib.hardware.sensor.encoders;

import frc.lib.hardware.HardwareRequirements;

public interface Encoder extends HardwareRequirements {

  Encoder config();

  double getPositoin();

  double getVelocity();

  void setGearRatio(double gearRatio);
}
