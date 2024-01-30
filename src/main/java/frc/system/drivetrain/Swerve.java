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

public class Swerve implements Drivetrain {
	final IMU imu;

	final SwerveDriveKinematics kinematics;
	final SwerveDrivePoseEstimator odometry;

	final List<Module> modules;

	public static class Module {
		final ProfiledMotor angle;
		final ProfiledMotor drive;
		final Translation2d offset;

		public Module(Translation2d offset, ProfiledMotor angle, ProfiledMotor drive) {
			this.offset = offset;
			this.angle = angle;
			this.drive = drive;
		}

		public Translation2d offset() {
			return this.offset;
		}

		public SwerveModulePosition position() {
			return new SwerveModulePosition(drive.position(), Rotation2d.fromRotations(angle.position()));
		}

		public SwerveModuleState state() {
			return new SwerveModuleState(drive.velocity(), Rotation2d.fromRotations(angle.position()));
		}

		public void setVelocity(SwerveModuleState state) {
			drive.setVelocity(state.speedMetersPerSecond);
			angle.setPosition(state.angle.getRotations());
		}

		public void setPosition(SwerveModuleState state) {
			drive.setPosition(state.speedMetersPerSecond);
			angle.setPosition(state.angle.getRotations());
		}
	}

	SwerveModulePosition[] positions() {
		return this.modules.stream().map(Module::position).toArray(SwerveModulePosition[]::new);
	}

	public Swerve(IMU imu, Module... modules) {
		this.imu = imu;
		this.modules = Arrays.asList(modules);

		kinematics = new SwerveDriveKinematics(
				this.modules.stream().map(Module::offset).toArray(Translation2d[]::new));

		odometry = new SwerveDrivePoseEstimator(kinematics, imu.yaw(), positions(), null);
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
			modules.get(i).setVelocity(states[i]);
	}

	@Override
	public void accept(Vision.Measurement t) {
		odometry.addVisionMeasurement(t.pose(), t.timestamp(), t.stdDev());
	}

	@Override
	public void set(Pose2d pose) {
		odometry.resetPosition(imu.yaw(), positions(), pose);
	}

	@Override
	public void periodic() {
		odometry.update(imu.yaw(), modules.stream().map(Module::position).toArray(SwerveModulePosition[]::new));
	}
}
