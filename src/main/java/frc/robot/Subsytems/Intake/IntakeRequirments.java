package frc.robot.Subsytems.Intake;

import frc.robot.Subsytems.SubsytemRequirments;

public interface IntakeRequirments extends SubsytemRequirments {

  // Controling the hardware
  public void setOutakeVoltage();

  public void setIntakeVoltage();

  public void off();

  /** Enable or disable brake mode on the intake. */
  public default void setBrakeMode(boolean enable) {
  }
}
