package frc.robot.Subsytems.Intake;

import frc.robot.Subsytems.SubsytemRequirments;

public interface IntakeRequirments extends SubsytemRequirments {

  // Controling the hardware
  void setOutakeSpeed();

  void setIntakeSpeed();

  boolean isOpen();

  boolean isClosed();
}
