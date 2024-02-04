package frc.system;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem, Consumer<Vision.Measurement> {
	/** Get the current drivetrain pose. */
	Pose2d pose();

	/** Resets the drivetrain pose. */
	void reset(Pose2d pose);

	/** Drives at the provided velocities. */
	void driveAt(ChassisSpeeds velocity);

	/** Drive to the provided position. */
	void driveTo(ChassisSpeeds position);
}
