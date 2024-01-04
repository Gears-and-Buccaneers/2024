package frc.lib.hardware.Motors.MotorControllers;

import frc.lib.hardware.HardwareRequirements;

public interface MotorController extends HardwareRequirements {

  MotorController config(int canID);

  void runVolts(double num);

  void brakeMode(boolean enable);

  void setInverted(boolean enable);

  int getCanID();

  void disable();

  void currentLimit();
}
