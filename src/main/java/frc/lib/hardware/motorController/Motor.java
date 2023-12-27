package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface Motor extends AutoCloseable, LoggableInputs {
  void runPercentOut(int num);

  void brakeMode(boolean enable);

  void inverted(boolean enable);

  int getCanID();

  default void toLog(LogTable table) {
  }

  default void fromLog(LogTable table) {
  }
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
