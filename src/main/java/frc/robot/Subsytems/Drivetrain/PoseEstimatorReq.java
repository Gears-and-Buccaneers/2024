package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsytems.SubsytemRequirments;

public interface PoseEstimatorReq extends SubsytemRequirments {
  Pose2d getPose2d();

  void resetEstimator(Pose2d pose);
}
