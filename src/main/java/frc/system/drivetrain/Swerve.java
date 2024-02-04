package frc.system.drivetrain;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.hardware.IMU;
import frc.hardware.ProfiledMotor;
import frc.system.Drivetrain;
import frc.system.Vision;

/** A Swerve drivetrain. */
public class Swerve implements Drivetrain {
	/** Gyroscope used for odometry. */
	final IMU imu;

	/** Inverse kinematics calculator. */
	final SwerveDriveKinematics kinematics;
	/** Odometry pose estimator with Kalman filter for vision measurements. */
	final SwerveDrivePoseEstimator odometry;

	/** All modules in this drivetrain. */
	final List<Module> modules;

	/** An individual Swerve module. */
	public static class Module {
		/** The motors of this Swerve module. */
		final ProfiledMotor angle, drive;
		/** The offset position from the robot origin that this module is placed at. */
		final Translation2d offset;

		/**
		 * Constructs a new module.
		 *
		 * @param offset The ofset position from the robot origin of this module.
		 * @param angle  The motor controlling the angle of the module wheel.
		 * @param drive  The motor controlling the module wheel.
		 */
		public Module(Translation2d offset, ProfiledMotor angle, ProfiledMotor drive) {
			this.offset = offset;
			this.angle = angle;
			this.drive = drive;
		}

		/** Gets the current position state of this module. */
		public SwerveModulePosition position() {
			return new SwerveModulePosition(drive.position(), Rotation2d.fromRotations(angle.position()));
		}

		/** Gets the current velocity state of this module. */
		public SwerveModuleState state() {
			return new SwerveModuleState(drive.velocity(), Rotation2d.fromRotations(angle.position()));
		}

		/** Sets this module to run at the provided velocity. */
		public void setVelocity(SwerveModuleState state) {
			drive.setVelocity(state.speedMetersPerSecond);
			angle.setPosition(state.angle.getRotations());
		}

		/** Sets this module to drive to the provided position. */
		public void setPosition(SwerveModuleState state) {
			drive.setRelativePosition(drive.position() + state.speedMetersPerSecond);
			angle.setPosition(state.angle.getRotations());
		}
	}

	/**
	 * Constructs a new Swerve drivetrain.
	 *
	 * @param imu     The gyroscope used for odometry.
	 * @param modules The individual modules in this drivetrain.
	 */
	public Swerve(IMU imu, Module... modules) {
		this.imu = imu;
		this.modules = Arrays.asList(modules);

		kinematics = new SwerveDriveKinematics(
				this.modules.stream().map(x -> x.offset).toArray(Translation2d[]::new));

		odometry = new SwerveDrivePoseEstimator(kinematics, imu.yaw(), positions(), null);
	}

	/** Collects the positions of the modules in */
	SwerveModulePosition[] positions() {
		return this.modules.stream().map(Module::position).toArray(SwerveModulePosition[]::new);
	}

	@Override
	public void driveAt(ChassisSpeeds velocity) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(velocity);

		for (int i = 0; i < states.length; i++)
			modules.get(i).setVelocity(states[i]);
	}

	@Override
	public void driveTo(ChassisSpeeds position) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(position);

		for (int i = 0; i < states.length; i++)
			modules.get(i).setPosition(states[i]);
	}

	@Override
	public Pose2d pose() {
		return odometry.getEstimatedPosition();
	}

	@Override
	public void reset(Pose2d pose) {
		odometry.resetPosition(imu.yaw(), positions(), pose);
	}

	/** Consume and process a vision measurement. */
	@Override
	public void accept(Vision.Measurement t) {
		// TODO: update positions of Swerve modules.
		odometry.addVisionMeasurement(t.pose(), t.timestamp(), t.stdDev());
	}

	@Override
	public void periodic() {
		odometry.update(imu.yaw(), modules.stream().map(Module::position).toArray(SwerveModulePosition[]::new));
	}
}
