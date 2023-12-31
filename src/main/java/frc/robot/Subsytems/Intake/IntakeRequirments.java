package frc.robot.Subsytems.Intake;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Subsytems.SubsytemRequirments;

public interface IntakeRequirments extends SubsytemRequirments, Sendable {

  // Controling the hardware
  void setOutakeSpeed();

  void setIntakeSpeed();
}
