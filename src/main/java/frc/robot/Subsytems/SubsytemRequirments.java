package frc.robot.Subsytems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsytemRequirments extends AutoCloseable, LoggableInputs {

  default void loadPreferences() {
  }

  /** Enable or disable brake mode on the intake. */
  default void setBrakeMode(boolean enable) {
  }

  void toLog(LogTable table);

  @Override
  default void fromLog(LogTable table) {
  }

  void setLogName(String logName);
}
