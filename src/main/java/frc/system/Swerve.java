package frc.system;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.config.SwerveConfig;
import frc.system.Vision.Measurement;

public class Swerve extends SwerveDrivetrain implements Subsystem, Consumer<Vision.Measurement> {
	private boolean hasPose = false;

	private final PathConstraints constraints;

	private final double angleDeadbandRadians = Units.degreesToRadians(5);

	private final SwerveRequest.SwerveDriveBrake cachedBrake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentric cachedFieldCentric = new SwerveRequest.FieldCentric();
	private final SwerveRequest.FieldCentricFacingAngle cachedFieldCentricFacing = new SwerveRequest.FieldCentricFacingAngle();
	private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

	private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();

	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private StructPublisher<Pose2d> ntPose = NetworkTableInstance.getDefault()
			.getStructTopic("Subsystems/Drivetrain/Pose", new Pose2dStruct()).publish();

	/**
	 * The rotation override for the drivetrain. Set to null if there is no
	 * override.
	 */
	private Rotation2d rotationOverride;

	public Swerve(
			PathConstraints constraints,
			double kSpeedAt12VoltsMps,
			SwerveDrivetrainConstants driveTrainConstants,
			SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		this.constraints = constraints;

		registerTelemetry((s) -> {
			ntPose.set(s.Pose);
			state.set(s);
		});

		zeroGyro();
		configurePathPlanner();
		register();

		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	public void accept(Measurement t) {
		if (hasPose) {
			if (t.stdDev() != null) {
				addVisionMeasurement(t.pose().toPose2d(), t.timestamp(), t.stdDev());
			} else {
				addVisionMeasurement(t.pose().toPose2d(), t.timestamp());
			}
		} else {
			seedFieldRelative(t.pose().toPose2d());
			hasPose = true;
		}
	}

	public SendableChooser<Command> getAutoPaths() {
		return null;
		// return AutoBuilder.buildAutoChooser();
	}

	public Pose2d pose() {
		return state.get().Pose;
	}

	public boolean hasPose() {
		return hasPose;
	}

	private ChassisSpeeds getCurrentRobotChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});

		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	private void configurePathPlanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : m_moduleLocations) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		AutoBuilder.configureHolonomic(
				this::pose, // Supplier of current robot pose
				this::seedFieldRelative, // Consumer for seeding pose against auto
				this::getCurrentRobotChassisSpeeds,
				(speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
				// robot
				new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
						new PIDConstants(10, 0, 0),
						SwerveConfig.kSpeedAt12VoltsMps,
						driveBaseRadius,
						new ReplanningConfig()),
				() -> {

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				}, // Change this if the path needs to be flipped on red vs blue
				this); // Subsystem for requirements
		// PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
	}

	private Optional<Rotation2d> getRotationTargetOverride() {
		return Optional.ofNullable(rotationOverride);
	}

	// Commands
	private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	public Command zeroGyro() {
		return runOnce(() -> {
			hasPose = false;
			seedFieldRelative(
					new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218), Rotation2d.fromDegrees(0)));
		});

		// return runOnce(() -> {
		// hasPose = false;
		// seedFieldRelative(
		// new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218),
		// Rotation2d.fromDegrees(270)));
		// });
	}

	public Command driveTo(Pose2d target, double velocity) {
		return AutoBuilder.pathfindToPose(target, constraints, velocity);
	}

	public Command DriveToThenPath(PathPlannerPath path) {

		return AutoBuilder.pathfindThenFollowPath(path, constraints, 0);
	}

	/**
	 * Values are squared!!!! this adds more perdition at low speeds
	 *
	 * @param xVel [-1,1] percent speed + is forward
	 * @param yVel [-1,1] percent speed + is Left
	 * @param rVel [-1, 1] percent angler rate + CC+ // TODO: check this value
	 * @return
	 */
	public Command controllerDrive(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rVel) {
		return applyRequest(() -> {
			double x = xVel.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
			double y = yVel.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;

			if (rotationOverride == null)
				return cachedFieldCentric.withVelocityX(x).withVelocityY(y)
						.withRotationalRate(rVel.getAsDouble() * SwerveConfig.kMaxAngularRate);
			else
				return cachedFieldCentricFacing.withVelocityX(x).withVelocityY(y).withTargetDirection(rotationOverride);
		});
	}

	/**
	 * Sets a rotational override. Provide a value of `null` to clear the override.
	 */
	public void setOverride(Rotation2d rotation) {
		rotationOverride = rotation;
	}

	/** Returns whether the drivetrain is facing the stored rotational override. */
	public boolean isAimed() {
		return rotationOverride != null
				&& Math.abs(pose().getRotation().getRadians() - rotationOverride.getRadians()) <= angleDeadbandRadians;
	}

	public Command brake() {
		return applyRequest(
				() -> cachedBrake);
	}
}
