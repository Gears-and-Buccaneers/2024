package frc.robot.Subsytems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface Module extends AutoCloseable{
    public SwerveModulePosition[] modulePositions();

    void set(SwerveModuleState[] state);

    SwerveModuleState[] get();

    SwerveModulePosition[] getPos();
}
