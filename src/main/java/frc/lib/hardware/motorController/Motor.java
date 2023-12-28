package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class Motor implements AutoCloseable, LoggableInputs {
  protected int canID;

  public Motor(int canID) {
    this.canID = canID;
  }

  abstract void runPercentOut(int num);

  abstract void brakeMode(boolean enable);

  abstract void inverted(boolean enable);

  public int getCanID() {
    return canID;
  }

  public void toLog(LogTable table) {
  }

  public void fromLog(LogTable table) {
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
