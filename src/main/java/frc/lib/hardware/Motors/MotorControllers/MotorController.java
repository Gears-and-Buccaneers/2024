package frc.lib.hardware.Motors.MotorControllers;

import frc.lib.hardware.HardwareRequirements;

public interface MotorController extends HardwareRequirements {

  MotorController build(int canID);

  // Setting
  void runVolts(double num);

  void brakeMode(boolean enable);

  void disable();

  // Getting
  int getCanID();

  double getAppliedVolts();

  double getCurentAmps();

  double getBusVoltage();

  // Config
  MotorController setInverted(boolean enable);

  MotorController setCurrentLimit(double CurrentLimit);

  MotorController setSimulated(double simulated);
}
