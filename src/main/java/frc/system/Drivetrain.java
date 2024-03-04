package frc.system;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem, Consumer<Vision.Measurement> {
    /** Get the current drivetrain pose. */
    Pose2d pose();

    /** Resets the drivetrain pose. */
    void reset(Pose2d pose);

    /** Puts the drivetrain in brake mode. */
    Command brake();

    /**
     * Drives at the provided velocities.
     *
     * @param rVel the rotation velocity in radians.
     */
    Command drive(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rVel);

    /** Drives at the provided velocity, facing the provided direction. */
    Command driveFacing(DoubleSupplier xVel, DoubleSupplier yVel, Rotation2d target);

    /** Drive to the provided position, ending at the provided velocity. */
    Command driveTo(Pose2d position, double velocity);

    Command DriveToThenPath(PathPlannerPath path);
}
