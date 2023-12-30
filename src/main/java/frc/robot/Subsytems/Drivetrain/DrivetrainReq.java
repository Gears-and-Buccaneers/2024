package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsytems.SubsytemRequirments;

public interface DrivetrainReq extends SubsytemRequirments {

  void setChassisSpeed(ChassisSpeeds targetVelocity);

  ChassisSpeeds getChassisSpeed();

  Rotation2d getAngle();
}
