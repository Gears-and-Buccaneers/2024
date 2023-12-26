package frc.robot.Subsytems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.sensor.imu.*;

public class DrivetrainIOSwerve implements DrivetrainRequirments {
  public IMU imu;

  public SwerveDriveKinematics kinematics;
  public SwerveDriveOdometry odometry;

  public double maxSpeed = 4.0;
  public double absoluteMaxSpeed = 4.0;
  public double maxAcceleration = 3.0;
  public double maxAngularVelocity = 8.0;
  public double maxAngularAcceleration = 4.0;
  public double drivetrainRadius = 0.4; // meaters from center fathest out point

  /** Locations of the wheels relative to the robot center. FL, FR, BL, BR meaters */
  Translation2d[] wheelLocations = {
    new Translation2d(0.265, 0.265),
    new Translation2d(0.265, -0.265),
    new Translation2d(-0.265, 0.265),
    new Translation2d(-0.265, -0.265)
  };

  private Module modules;

  public DrivetrainIOSwerve(SubsystemBase subsystemBase) {
    System.out.println("[Init] Creating " + this.getClass().getSimpleName());

    // IMU configs
    imu = new Pigeon(0);

    initPrefrences();

    modules = new SwerveMod(0);
    kinematics =
        new SwerveDriveKinematics(
            wheelLocations[0], wheelLocations[1], wheelLocations[2], wheelLocations[3]);

    odometry = new SwerveDriveOdometry(kinematics, imu.getYaw(), modules.modulePositions());
  }

  // Chassis Speeds ------------------------
  @Override
  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(modules.get());
  }

  @Override
  public void setChassisSpeed(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, absoluteMaxSpeed);

    modules.set(moduleStates);
  }

  @Override
  public void stopChassis() {}

  // odometry-------------------------------------------
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(imu.getYaw(), modules.getPos(), pose);
  }

  @Override
  public void updateOdometry() {
    odometry.update(imu.getYaw(), modules.getPos());
  }

  @Override
  public void zeroGyro() {
    imu.zeroIMU();
  }

  @Override
  public void configAutoBuilder(SubsystemBase subsystemBase) {
    AutoBuilder.configureHolonomic(
        this::getPose2d, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your
            // Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            maxSpeed, // Max module speed, in m/s
            drivetrainRadius, // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        subsystemBase // Reference to this subsystem to set requirements
        );
  }

  private void initPrefrences() {
    Preferences.initDouble("maxSpeed", maxSpeed);
  }

  // auto sendable and auto closable
  public void loadPreferences() {
    maxSpeed = Preferences.getDouble("maxSpeed", maxSpeed);
  }

  @Override
  public void updateInputs(DrivetrainInputs inputs) {
    inputs.GyroConnected = imu.connected();
  }

  @Override
  public void close() throws Exception {
    imu.close();
    modules.close();
  }
}
