package frc.system;

import java.util.function.Consumer;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem, Consumer<Vision.Measurement> {
    /** Get the current drivetrain pose. */
    Pose2d getPose();

    /** Resets the drivetrain pose. */
    void reset(Pose2d pose);

    /**
     * Drives at the provided velocities.
     * 
     * @param rVel the rotation velocity in radians.
     */
    Command drive(double xVel, double yVel, double rVel);

    /** Drives at the provided velocity, facing the provided direction. */
    Command driveFacing(double xVel, double yVel, Rotation2d target);

    /** Drive to the provided position, ending at the provided velocity. */
    Command driveTo(Pose2d position, double velocity);

    Command xState();

    Command zeroGyro();

    SendableChooser<Command> getAutoPaths();

    Command DriveToThenPath(PathPlannerPath path, double velocity);

    void log();
}
