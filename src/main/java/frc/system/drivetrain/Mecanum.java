package frc.system.drivetrain;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.hardware.IMU;
import frc.hardware.ProfiledMotor;
import frc.system.Drivetrain;
import frc.system.Vision;

/** A Mecanum drivetrain. */
public class Mecanum implements Drivetrain {
	/** Gyroscope used for odometry. */
	final IMU imu;

	/** Inverse kinematics calculator. */
	final MecanumDriveKinematics kinematics;
	/** Odometry pose estimator with Kalman filter for vision measurements. */
	final MecanumDrivePoseEstimator odometry;

	/** All modules in this drivetrain. */
	final ProfiledMotor frontLeft, frontRight, rearLeft, rearRight;

	/** An individual Mecanum module. */
	public static class Module {
		/** The motor driving the Mecanum module wheel. */
		final ProfiledMotor motor;
		/** The offset position from the robot origin that this module is placed at. */
		final Translation2d offset;

		/**
		 * Constructs a new module.
		 *
		 * @param offset The ofset position from the robot origin of this module.
		 * @param motor  The motor of this Mecanum module.
		 */
		public Module(Translation2d offset, ProfiledMotor motor) {
			this.offset = offset;
			this.motor = motor;
		}
	}

	/**
	 * Constructs a new Mecanum drivetrain.
	 *
	 * @param imu     The gyroscope used for odometry.
	 * @param modules The individual modules in this drivetrain.
	 */
	public Mecanum(IMU imu, Module frontLeft, Module frontRight, Module rearLeft, Module rearRight) {
		this.imu = imu;

		this.frontLeft = frontLeft.motor;
		this.frontRight = frontRight.motor;
		this.rearLeft = rearLeft.motor;
		this.rearRight = rearRight.motor;

		kinematics = new MecanumDriveKinematics(frontLeft.offset, frontRight.offset, rearLeft.offset, rearRight.offset);

		odometry = new MecanumDrivePoseEstimator(kinematics, imu.yaw(), positions(), null);
	}

	/** Collects the positions of the modules */
	MecanumDriveWheelPositions positions() {
		return new MecanumDriveWheelPositions(frontLeft.position(), frontRight.position(),
				rearLeft.position(), rearRight.position());
	}

	@Override
	public void driveAt(ChassisSpeeds velocity) {
		MecanumDriveWheelSpeeds states = kinematics.toWheelSpeeds(velocity);

		frontLeft.setVelocity(states.frontLeftMetersPerSecond);
		frontRight.setVelocity(states.frontRightMetersPerSecond);
		rearLeft.setVelocity(states.rearLeftMetersPerSecond);
		rearRight.setVelocity(states.rearRightMetersPerSecond);
	}

	@Override
	public void driveTo(ChassisSpeeds position) {
		MecanumDriveWheelSpeeds states = kinematics.toWheelSpeeds(position);

		frontLeft.setRelativePosition(states.frontLeftMetersPerSecond);
		frontRight.setRelativePosition(states.frontRightMetersPerSecond);
		rearLeft.setRelativePosition(states.rearLeftMetersPerSecond);
		rearRight.setRelativePosition(states.rearRightMetersPerSecond);
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
		// TODO: update positions of Mecanum modules.
		odometry.addVisionMeasurement(t.pose(), t.timestamp(), t.stdDev());
	}

	@Override
	public void periodic() {
		odometry.update(imu.yaw(), positions());
	}
}
