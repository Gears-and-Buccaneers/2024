package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSwerve implements DrivetrainReq {

  private final SwerveModule[] swerveModules;
  private final SwerveModulePosition[] swerveModulePositions;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  private ChassisSpeeds targetVelocity = new ChassisSpeeds();

  private final double wheelbase = Units.inchesToMeters(24);
  private final double track = Units.inchesToMeters(24);

  private final double maxVelocityMetersPerSec;
  private final double maxAngularVelocityRadPerSec;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public DrivetrainSwerve() {
    maxVelocityMetersPerSec = 4.3;
    maxAngularVelocityRadPerSec = maxVelocityMetersPerSec / Math.hypot(wheelbase / 2, track / 2);

    swerveModules = new SwerveModule[] {
        new SwerveModule(1, 2, "FrontLeft"),
        new SwerveModule(3, 4, "FrontRight"),
        new SwerveModule(5, 6, "BackLeft"),
        new SwerveModule(7, 8, "BackRight")
    };
    swerveModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    Translation2d frontLeftLocation = new Translation2d(wheelbase / 2, track / 2);
    Translation2d frontRightLocation = new Translation2d(wheelbase / 2, -track / 2);
    Translation2d backLeftLocation = new Translation2d(-wheelbase / 2, track / 2);
    Translation2d backRightLocation = new Translation2d(-wheelbase / 2, -track / 2);
    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    odometry = new SwerveDriveOdometry(kinematics, getAngle(), swerveModulePositions);
  }

  @Override
  public void setChassisSpeed(ChassisSpeeds targetVelocity) {
    this.targetVelocity = targetVelocity;
  }

  private Pose2d simOdometry = new Pose2d();
  double[] lastModulePositionsRad = { 0, 0, 0, 0 };
  private Field2d field;
  double angle;

  public void periodic() {
    loadPreferences();
    calcAngle();

    SwerveModuleState[] optimizedSwerveModuleStates = new SwerveModuleState[4];

    swerveModuleStates = kinematics.toSwerveModuleStates(targetVelocity);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocityMetersPerSec);

    synchronized (swerveModules) {
      for (int i = 0; i < optimizedSwerveModuleStates.length; i++) {
        optimizedSwerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i],
            swerveModules[i].getPosition().angle);
        swerveModules[i].setTargetState(optimizedSwerveModuleStates[i]);
      }
    }
    odometry.update(getAngle(), getSwerveModulePositions());
    Logger.recordOutput("Drive/Angle", (getAngle().getRadians() - angle) * 50);
    angle = getAngle().getRadians();
    Logger.recordOutput("Drive/AnglePS", getAngle().getRadians());
    Logger.recordOutput("Drive/ModuleStates", swerveModuleStates);
    Logger.recordOutput(
        "Drive/TargetChassisVelocity",
        new double[] {
            targetVelocity.vxMetersPerSecond,
            targetVelocity.vyMetersPerSecond,
            targetVelocity.omegaRadiansPerSecond
        });
    Logger.recordOutput("Drive/ModuleStatesString", test());
    Logger.recordOutput("Drive/OptimizedModuleStates", optimizedSwerveModuleStates);
    Logger.recordOutput("Drive/Odomaty", new double[] { odometry.getPoseMeters().getX(),
        odometry.getPoseMeters().getY(), getAngle().getRadians() });
    Logger.recordOutput("Drive/Odomaty2", new double[] {
        odometry.getPoseMeters().getX(),
        odometry.getPoseMeters().getY(),
        0,
        new Rotation3d(0, 0, angle).getQuaternion().getW(),
        new Rotation3d(0, 0, angle).getQuaternion().getX(),
        new Rotation3d(0, 0, angle).getQuaternion().getY(),
        new Rotation3d(0, 0, angle).getQuaternion().getZ() });
    Logger.recordOutput("Drive/Odomaty2String", odometry.getPoseMeters().toString());

  }

  public double[] test() {
    double[] testing = new double[] {
        swerveModuleStates[0].angle.getRadians(), swerveModuleStates[0].speedMetersPerSecond,
        swerveModuleStates[1].angle.getRadians(), swerveModuleStates[1].speedMetersPerSecond,
        swerveModuleStates[2].angle.getRadians(), swerveModuleStates[2].speedMetersPerSecond,
        swerveModuleStates[3].angle.getRadians(), swerveModuleStates[3].speedMetersPerSecond
    };
    return testing;
  }

  public SwerveModule[] getSwerveModules() {
    synchronized (swerveModules) {
      return swerveModules;
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        swerveModules[0].getPosition(),
        swerveModules[1].getPosition(),
        swerveModules[2].getPosition(),
        swerveModules[3].getPosition(),
    };
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public ChassisSpeeds getChassisSpeed() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeed'");
  }

  @Override
  public Rotation2d getAngle() {
    return simOdometry.getRotation();
  }

  private void calcAngle() {
    getSwerveModules();

    Rotation2d[] turnPositions = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      turnPositions[i] = getSwerveModules()[i].getPosition().angle;
    }

    SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStatesDiff[i] = new SwerveModuleState(
          (getSwerveModules()[i].getPosition().distanceMeters - lastModulePositionsRad[i])
              * Units.inchesToMeters(2),
          turnPositions[i]);
      lastModulePositionsRad[i] = getSwerveModules()[i].getPosition().distanceMeters;
    }

    simOdometry = simOdometry.exp(
        new Twist2d(
            getKinematics().toChassisSpeeds(measuredStatesDiff).vxMetersPerSecond,
            getKinematics().toChassisSpeeds(measuredStatesDiff).vyMetersPerSecond,
            getKinematics().toChassisSpeeds(measuredStatesDiff).omegaRadiansPerSecond * 10));
    Logger.recordOutput("rotationPerSec",
        getKinematics().toChassisSpeeds(measuredStatesDiff).omegaRadiansPerSecond * 10);
  }

  public void setBrakeMode(boolean enable) {
    synchronized (swerveModules) {
      for (SwerveModule module : swerveModules) {
        module.setBrakeMode(enable);
      }
    }
  }

  @Override
  public void disable() {
    synchronized (swerveModules) {
      for (SwerveModule module : swerveModules) {
        module.disable();
      }
    }
  }

  // -----------------------------
  @Override
  public void loadPreferences() {
  }

  @Override
  public void toLog(LogTable table) {
    synchronized (swerveModules) {
      for (SwerveModule module : swerveModules) {
        module.toLog(table);
      }
    }
    table.put("234", 123);
  }

  @Override
  public void close() throws Exception {
    synchronized (swerveModules) {
      for (SwerveModule module : swerveModules) {
        module.close();
      }
    }
  }
}
