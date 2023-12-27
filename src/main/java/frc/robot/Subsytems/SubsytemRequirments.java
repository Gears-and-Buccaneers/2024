package frc.robot.Subsytems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsytemRequirments extends AutoCloseable, LoggableInputs {
  void loadPreferences();

  /** Enable or disable brake mode on the intake. */
  default void setBrakeMode(boolean enable) {}
}
