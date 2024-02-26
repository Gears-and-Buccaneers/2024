package frc.system.drivetrain;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
	private final TrapezoidProfile profile;

	public CtreSwerve(double velMax, double accMax, SwerveDrivetrainConstants driveTrainConstants,
			SwerveModuleConstants... modules) {

		super(driveTrainConstants, modules);

		state = new AtomicReference<>();
		profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(velMax,
				accMax));

		registerTelemetry((s) -> state.set(s));
	}

	@Override
	public void reset(Pose2d pose) {
		seedFieldRelative(pose);
	}

	@Override
	public void accept(Measurement t) {
		addVisionMeasurement(t.pose(), t.timestamp(), t.stdDev());
	}

	@Override
	public Pose2d pose() {
		return state.get().Pose;
	}

	@Override
	public Command drive(Pose2d target, double velocity) {
		// TODO
		return null;

		// return new Command() {
		// Pose2d pose = target;
		// double vel = velocity;

		// double t;

		// @Override
		// public void initialize() {
		// t = 0;
		// }

		// @Override
		// public void execute() {
		// SwerveDriveState s = state.get();
		// ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(s.ModuleStates);

		// Translation2d err = s.Pose.minus(pSetpoint).getTranslation();

		// TrapezoidProfile.State velocity = profile.calculate(t++ / 50.0, new
		// TrapezoidProfile.State(),
		// new TrapezoidProfile.State(, vSetpoint));

		// setControl(new SwerveRequest.FieldCentric()
		// .withVelocityX(velocity.getX())
		// .withVelocityY(velocity.getY())
		// .withRotationalRate(velocity.getRotation().getRadians()));
		// }
		// };
	}

	@Override
	public Command drive(Transform2d velocity) {
		return new Command() {
			@Override
			public void initialize() {
				setControl(new SwerveRequest.FieldCentric()
						.withVelocityX(velocity.getX())
						.withVelocityY(velocity.getY())
						.withRotationalRate(velocity.getRotation().getRadians()));
			}
		};
	}
}
