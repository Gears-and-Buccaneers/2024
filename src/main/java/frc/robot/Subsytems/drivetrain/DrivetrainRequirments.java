package frc.robot.Subsytems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DrivetrainRequirments extends AutoCloseable {

    @AutoLog
    public static class DrivetrainInputs {
        public boolean GyroConnected = false;
        public Rotation2d yaw = new Rotation2d();
    }

    /** Updates the set of loggable inputs. */
    void updateInputs(DrivetrainInputs inputs);

    // ------------------------------------------

    void setChassisSpeed(ChassisSpeeds speeds);

    ChassisSpeeds getChassisSpeed();

    void stopChassis();

    void zeroGyro();

    void resetOdometry(Pose2d pose);

    void updateOdometry(Rotation2d yaw, double state);

    Pose2d getPose2d();

}
