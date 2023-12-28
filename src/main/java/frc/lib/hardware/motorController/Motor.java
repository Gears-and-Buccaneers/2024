package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.Logger;

import frc.lib.hardware.HardwareRequirments;

public interface Motor extends HardwareRequirments {

  void runPercentOut(int num);

  void brakeMode(boolean enable);

  void setInverted(boolean enable);

  int getCanID();

  void disable();

  default void logInputs(String subsytemName) {
    Logger.processInputs(subsytemName + "/motor" + getCanID(), this);
  }
}
