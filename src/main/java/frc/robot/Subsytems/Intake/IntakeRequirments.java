package frc.robot.Subsytems.Intake;

import frc.robot.Subsytems.SubsytemRequirments;

public interface IntakeRequirments extends SubsytemRequirments {

  // Controling the hardware
  void setOutakeVoltage();

  void setIntakeVoltage();

  void off();

  boolean isOpen();

  boolean isClosed();

  void periodic();
}
