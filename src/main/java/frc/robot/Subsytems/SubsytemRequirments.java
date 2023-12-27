package frc.robot.Subsytems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsytemRequirments extends AutoCloseable, LoggableInputs {
  void loadPreferences();
}
