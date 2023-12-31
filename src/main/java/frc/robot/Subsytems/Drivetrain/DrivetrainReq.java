package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsytems.SubsytemRequirments;

public interface DrivetrainReq extends SubsytemRequirments {

  /**
   * 
   * @param targetVelocity target VElocity Meters/second ROBOT RELIVIVE
   */
  void setChassisSpeed(ChassisSpeeds targetVelocity);

  /**
   * 
   * @return ROBOT RELITIVE in meters per second
   */
  ChassisSpeeds getChassisSpeed();

  Rotation2d getAngle();

  /**
   * 
   * @return the maximum module speed in meaters/second
   */
  double getMaxModuleSpeed();

  double getMaxModuleAccl();

  /**
   * 
   * @return max module spped in radians per second
   */
  double getMaxAngularVelocity();

  double getMaxAngularAccl();

  /**
   * 
   * @return the radius of the drivetrain (for auton)
   */
  double getRadius();

  // DELEAT JUST TESTING
  Pose2d getPose2d();

  void resetEstimator(Pose2d pose);
}
