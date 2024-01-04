package frc.robot.Subsystems.Intake;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Subsystems.SubsystemRequirements;

public interface IntakeRequirements extends SubsystemRequirements, Sendable {

  // Controling the hardware
  void setOuttakeSpeed();

  void setIntakeSpeed();
}
