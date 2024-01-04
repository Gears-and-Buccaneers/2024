package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.SubsystemRequirements;

public interface ArmRequirements extends SubsystemRequirements {

  void wristAngleSetpoint(Rotation2d angle);

  void elevatorAngleSetpoint(Rotation2d angle);

  void elevatorLengthSetpoint(double ft);

  boolean atSetpoint();
}
