package frc.robot.Subsytems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsytemRequirments extends AutoCloseable, LoggableInputs {

  void toLog(LogTable table);

  default void periodic() {}

  void disable();

  default void simulationPeriodic() {}

  default void loadPreferences() {}

  default void setBrakeMode(boolean enable) {}

  @Override
  default void fromLog(LogTable table) {}
}
