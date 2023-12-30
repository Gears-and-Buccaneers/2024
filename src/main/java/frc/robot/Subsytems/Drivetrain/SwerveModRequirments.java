package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModRequirments {

  default void setTargetSteerPosition(Rotation2d targetSteerPosition) {}
}
