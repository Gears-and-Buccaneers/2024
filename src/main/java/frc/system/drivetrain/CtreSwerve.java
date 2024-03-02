package frc.system.drivetrain;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Drivetrain;
import frc.system.Vision.Measurement;

public class CtreSwerve extends SwerveDrivetrain implements Drivetrain {
	private final SwerveRequest.FieldCentric cachedFieldCentric = new SwerveRequest.FieldCentric();
	private final SwerveRequest.FieldCentricFacingAngle cachedFieldCentricFacing = new SwerveRequest.FieldCentricFacingAngle();

	private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();
	// private final TrapezoidProfile profile;

	public CtreSwerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
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
	public Command driveTo(Pose2d target, double velocity) {
		// TODO: use pathplanner
		return null;
	}

	@Override
	public void drive(double xVel, double yVel, double rVel) {
		setControl(cachedFieldCentric.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rVel));
	}

	@Override
	public void driveFacing(double xVel, double yVel, Rotation2d target) {
		setControl(cachedFieldCentricFacing.withVelocityX(xVel).withVelocityY(yVel).withTargetDirection(target));
	}
}
