package frc.lib.hardware.motorController;

import frc.lib.hardware.HardwareRequirments;

public interface Motor extends HardwareRequirments {

  void runPercentOut(int num);

  void brakeMode(boolean enable);

  void setInverted(boolean enable);

  int getCanID();

  void disable();

  /**
   * Inverted
   * Coast or break
   * max percent output/voltage
   * min percent output/voltage
   * 
   * PID
   * p
   * i
   * d
   * s
   * v
   * a
   * 
   * Motion magic
   * max accelratoin
   * max decelratoin
   * cruse velocity
   * 
   * feed forward (optonal)
   * 
   * 
   * can take a encoder
   * set gear ratio
   * set encoder counts per tick
   * 
   * 
   * controll modes
   * percent output
   * voltage
   * velocty
   * 
   */
}
