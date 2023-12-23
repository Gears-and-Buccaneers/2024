package frc.robot.Subsytems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DrivetrainRequirments {

    public static class Module {

    }

    void setChassisSpeed(ChassisSpeeds speeds);

    ChassisSpeeds getChassisSpeed();

    void zeroGyro();

    void resetOdometry(Pose2d pose);

    Pose2d getPose2d();

}
