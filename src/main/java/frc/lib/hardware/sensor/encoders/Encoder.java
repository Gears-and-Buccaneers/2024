package frc.lib.hardware.sensor.encoders;

import frc.lib.hardware.HardwareRequirements;

public interface Encoder extends HardwareRequirements {

  void zero();

  double getPosition();

  double getVelocity();

  Encoder setGearRatio(double gearRatio);

  Encoder setInverted(boolean inverted);

  Encoder setOffset(double offset);
}
