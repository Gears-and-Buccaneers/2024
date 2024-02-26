package frc.system;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem, Consumer<Vision.Measurement> {
	/** Get the current drivetrain pose. */
	Pose2d pose();

	/** Resets the drivetrain pose. */
	void reset(Pose2d pose);

	/** Drives at the provided velocities. */
	Command drive(Transform2d velocity);

	/** Drive to the provided position, ending at the provided velocity. */
	Command drive(Pose2d position, double velocity);
}
