package frc.lib.hardware.motorController;

public interface Motor extends AutoCloseable {
  void runPercentOut(int num);
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
