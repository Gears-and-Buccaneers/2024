package frc.robot.Subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.SubsystemRequirements;

public interface PoseEstimatorReq extends SubsystemRequirements {
  Pose2d getPose2d();

  void resetEstimator(Pose2d pose);
}
