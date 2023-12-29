package frc.lib.hardware;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface HardwareRequirments extends AutoCloseable, LoggableInputs {

  boolean connected();

  default void toLog(LogTable table, String path) {}

  default void fromLog(LogTable table) {}

  default void toLog(LogTable table) {}
}
