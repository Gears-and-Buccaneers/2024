package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.Logger;

import frc.lib.hardware.HardwareRequirments;

public interface Motor extends HardwareRequirments {

  void runPercentOut(double num);

  void brakeMode(boolean enable);

  void setInverted(boolean enable);

  int getCanID();

  void disable();
}
