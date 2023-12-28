package frc.robot.Subsytems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsytemRequirments extends AutoCloseable, LoggableInputs {

  void toLog(LogTable table);

  void setSimpleName(String SimpleName);

  void periodic();

  void disable();

  default void loadPreferences() {
  }

  default void setBrakeMode(boolean enable) {
  }

  @Override
  default void fromLog(LogTable table) {
  }
}
