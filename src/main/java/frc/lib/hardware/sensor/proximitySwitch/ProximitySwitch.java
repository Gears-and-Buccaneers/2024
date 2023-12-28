package frc.lib.hardware.sensor.proximitySwitch;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.HardwareRequirments;

public interface ProximitySwitch extends HardwareRequirments {
  /**
   * @return true if open false if closed
   */
  boolean get();

  /**
   * @return true if closed false if open
   */
  public default boolean isClosed() {
    return !get();
  }

  /**
   * @return true if open false if closed
   */
  public default boolean isOpen() {
    return get();
  }

  public default Trigger trigger() {
    return new Trigger(
        () -> {
          return isOpen();
        });
  }
}
