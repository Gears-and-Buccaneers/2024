package frc.system;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem, Consumer<Vision.Measurement> {
	void set(Pose2d pose);

	void driveAt(ChassisSpeeds velocity);

	void driveTo(ChassisSpeeds position);
}
