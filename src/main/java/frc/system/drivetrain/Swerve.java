// package frc.system.drivetrain;

// import java.util.Arrays;
// import java.util.List;
// import java.util.function.Function;

// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.struct.Pose2dStruct;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.networktables.StructPublisher;
// import frc.hardware.IMU;
// import frc.hardware.ProfiledMotor;
// import frc.system.Drivetrain;
// import frc.system.Vision;

// /** A Swerve drivetrain. */
// public class Swerve implements Drivetrain {
// final StructArrayPublisher<SwerveModuleState> ntSetpoints;
// final StructArrayPublisher<SwerveModuleState> ntMeasured;
// final StructPublisher<Pose2d> ntPose;

// /** Gyroscope used for odometry. */
// final IMU imu;

// /** Inverse kinematics calculator. */
// final SwerveDriveKinematics kinematics;
// /** Odometry pose estimator with Kalman filter for vision measurements. */
// final SwerveDrivePoseEstimator odometry;

// /** All modules in this drivetrain. */
// final List<Module> modules;

// /** An individual Swerve module. */
// public static class Module {
// /** The motors of this Swerve module. */
// final ProfiledMotor drive, angle;
// /** The offset position from the robot origin that this module is placed at.
// */
// final Translation2d offset;

// /**
// * Constructs a new module.
// *
// * @param offset The offset position of this module from the robot origin.
// * @param angle The motor controlling the angle of the module wheel.
// * @param drive The motor controlling the module wheel.
// */
// public Module(String name, Translation2d offset, ProfiledMotor drive,
// ProfiledMotor angle) {
// this.offset = offset;
// this.drive = drive;
// this.angle = angle;
// }

// /** Gets the current position state of this module. */
// public SwerveModulePosition position() {
// return new SwerveModulePosition(drive.position(),
// Rotation2d.fromRotations(angle.position()));
// }

// /** Gets the current velocity state of this module. */
// public SwerveModuleState state() {
// return new SwerveModuleState(drive.velocity(),
// Rotation2d.fromRotations(angle.position()));
// }

// /** Sets this module to run at the provided velocity. */
// public void setVelocity(SwerveModuleState state) {
// System.out.println(state.speedMetersPerSecond + ", " +
// state.angle.getDegrees());

// drive.setVelocity(state.speedMetersPerSecond);
// angle.setPosition(state.angle.getRotations());
// }

// /** Sets this module to drive to the provided position. */
// public void setPosition(SwerveModuleState state, ) {
// drive.setRelativePosition(state.speedMetersPerSecond);
// angle.setPosition(state.angle.getRotations());
// }
// }

// /**
// * Constructs a new Swerve drivetrain.
// *
// * @param imu The gyroscope used for odometry.
// * @param modules The individual modules in this drivetrain.
// */
// public Swerve(IMU imu, Module... modules) {
// this.imu = imu;
// this.modules = Arrays.asList(modules);

// kinematics = new SwerveDriveKinematics(collect(x -> x.offset));
// odometry = new SwerveDrivePoseEstimator(kinematics, imu.yaw(),
// collect(Module::position), new Pose2d());

// NetworkTableInstance nt = NetworkTableInstance.getDefault();

// ntSetpoints = nt.getStructArrayTopic("Swerve/Setpoints", new
// SwerveModuleStateStruct()).publish();
// ntMeasured = nt.getStructArrayTopic("Swerve/Measured", new
// SwerveModuleStateStruct()).publish();
// ntPose = nt.getStructTopic("Swerve/Pose", new Pose2dStruct()).publish();

// register();
// }

// @SuppressWarnings("unchecked")
// <T> T[] collect(Function<Module, T> fn) {
// return (T[]) modules.stream().map(fn).toArray(Object[]::new);
// }

// @Override
// public void drive(Transform2d vel) {
// ChassisSpeeds speeds = new ChassisSpeeds(vel.getX(), vel.getY(),
// vel.getRotation().getRadians());

// SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

// ntSetpoints.set(states);

// for (int i = 0; i < states.length; i++)
// modules.get(i).setVelocity(states[i]);
// }

// @Override
// public void drive(Pose2d position, Transform2d velocity) {
// Transform2d pose = position.minus(pose());
// ChassisSpeeds speeds = new ChassisSpeeds(pose.getX(), pose.getY(),
// pose.getRotation().getRadians());

// SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

// ntSetpoints.set(states);

// for (int i = 0; i < states.length; i++)
// modules.get(i).setPosition(states[i]);
// }

// @Override
// public Pose2d pose() {
// return odometry.getEstimatedPosition();
// }

// @Override
// public void reset(Pose2d pose) {
// odometry.resetPosition(imu.yaw(), collect(Module::position), pose);
// }

// /** Consume and process a vision measurement. */
// @Override
// public void accept(Vision.Measurement t) {
// // TODO: update positions of Swerve modules.
// odometry.addVisionMeasurement(t.pose(), t.timestamp(), t.stdDev());
// }

// @Override
// public void periodic() {
// odometry.update(imu.yaw(), collect(Module::position));

// ntMeasured.set(collect(Module::state));
// ntPose.set(odometry.getEstimatedPosition());
// }
// }
