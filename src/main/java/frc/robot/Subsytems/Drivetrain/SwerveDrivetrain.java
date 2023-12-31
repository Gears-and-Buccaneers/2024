package frc.robot.Subsytems.Drivetrain;

import org.littletonrobotics.junction.LogTable;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveDrivetrain implements DrivetrainReq {
  private SimSwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private SimGyro gyro;

  private Field2d field = new Field2d();

  public SwerveDrivetrain() {
    gyro = new SimGyro();
    modules = new SimSwerveModule[] {
        new SimSwerveModule(),
        new SimSwerveModule(),
        new SimSwerveModule(),
        new SimSwerveModule()
    };
    kinematics = new SwerveDriveKinematics(
        new Translation2d(0.4, 0.4),
        new Translation2d(0.4, -0.4),
        new Translation2d(-0.4, 0.4),
        new Translation2d(-0.4, -0.4));

    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    gyro.updateRotation(getChassisSpeed().omegaRadiansPerSecond);

    odometry.update(getAngle(), getPositions());

    field.setRobotPose(getPose2d());
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetEstimator(Pose2d pose) {
    odometry.resetPosition(getAngle(), getPositions(), pose);
  }

  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose2d().getRotation()));
  }

  public void setChassisSpeed(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, getMaxModuleSpeed());

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Basic simulation of a swerve module, will just hold its current state and not
   * use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(
          currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any
   * hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps) {
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("you dumb", 10);
    table.put("2dOdomaty",
        new double[] { getPose2d().getX(), getPose2d().getY(), getPose2d().getRotation().getRadians() });
    table.put("3dOdomaty",
        new double[] {
            getPose2d().getX(),
            getPose2d().getY(),
            0,
            new Rotation3d(0, 0, getPose2d().getRotation().getRadians()).getQuaternion().getW(),
            new Rotation3d(0, 0, getPose2d().getRotation().getRadians()).getQuaternion().getX(),
            new Rotation3d(0, 0, getPose2d().getRotation().getRadians()).getQuaternion().getY(),
            new Rotation3d(0, 0, getPose2d().getRotation().getRadians()).getQuaternion().getZ() });

  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'disable'");
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public Rotation2d getAngle() {
    return gyro.getRotation2d();
  }

  @Override
  public double getMaxModuleSpeed() {
    return 4;
  }

  @Override
  public double getMaxModuleAccl() {
    return 4;
  }

  @Override
  public double getMaxAngularVelocity() {
    return 4;
  }

  @Override
  public double getMaxAngularAccl() {
    return 4;
  }

  @Override
  public double getRadius() {
    return .5;
  }
}