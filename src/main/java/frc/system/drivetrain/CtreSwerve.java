package frc.system.drivetrain;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Drivetrain;
import frc.system.Vision.Measurement;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CtreSwerve extends SwerveDrivetrain implements Drivetrain {
	private final AtomicReference<SwerveDriveState> state;
	// private final TrapezoidProfile profile;

	public CtreSwerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);

		state = new AtomicReference<>();

		registerTelemetry((s) -> state.set(s));
	}

	@Override
	public void reset(Pose2d pose) {
		seedFieldRelative(pose);
	}

	@Override
	public void accept(Measurement t) {
		if (t.stdDev() != null) {
			addVisionMeasurement(t.pose().toPose2d(), t.timestamp(), t.stdDev());
		} else {
			addVisionMeasurement(t.pose().toPose2d(), t.timestamp());
		}
	}

	@Override
	public Pose2d pose() {
		return state.get().Pose;
	}

	@Override
	public Command drive(Pose2d target, double velocity) {
		// TODO: use pathplanner
		return null;
	}

	@Override
	public void drive(Transform2d velocity) {
		setControl(new SwerveRequest.FieldCentric()
				.withVelocityX(velocity.getX())
				.withVelocityY(velocity.getY())
				.withRotationalRate(velocity.getRotation().getRadians()));
	}

	@Override
	public void driveFacing(Transform2d velocity) {
		setControl(new SwerveRequest.FieldCentric()
				.withVelocityX(velocity.getX())
				.withVelocityY(velocity.getY())
				.withRotationalRate(velocity.getRotation().getRadians()));
	}
}
