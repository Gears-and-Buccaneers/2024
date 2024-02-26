// package frc.config;

// import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;

// import edu.wpi.first.math.geometry.Translation2d;
// import frc.Config;
// import frc.hardware.imu.Pigeon;
// import frc.hardware.profiledmotor.FX;
// import frc.system.Drivetrain;
// import frc.system.drivetrain.Swerve;

// public class SwerveConfig implements Config {
// @Override
// public Drivetrain drivetrain() {
// Slot0Configs steerGains = new Slot0Configs()
// .withKP(100).withKI(0).withKD(0.2)
// .withKS(0).withKV(1.5).withKA(0);

// Slot0Configs driveGains = new Slot0Configs()
// .withKP(3).withKI(0).withKD(0)
// .withKS(0).withKV(0).withKA(0);

// CurrentLimitsConfigs currentLimits = new
// CurrentLimitsConfigs().withSupplyCurrentLimit(1)
// .withSupplyCurrentLimitEnable(true);

// FeedbackConfigs encoder = new FeedbackConfigs().withRotorToSensorRatio(-150 /
// 7)
// .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

// ClosedLoopGeneralConfigs general = new ClosedLoopGeneralConfigs();
// general.ContinuousWrap = true;

// TalonFXConfiguration angle = new TalonFXConfiguration()
// .withSlot0(new Slot0Configs().withKP(0.6).withKI(0.0).withKD(12.0))
// .withCurrentLimits(currentLimits)
// .withClosedLoopGeneral(general);

// TalonFXConfiguration drive = new TalonFXConfiguration()
// .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(8.14))
// .withCurrentLimits(currentLimits)
// .withSlot0(new Slot0Configs().withKP(0.1).withKI(0.0).withKD(12.0));

// // Forward is positive X, left is positive Y.
// return new Swerve(new Pigeon(1),
// new Swerve.Module("FrontLeft", new Translation2d(5.875, 10.875),
// new FX(1).with(drive),
// new FX(2).with(angle).with(encoder.withFeedbackRemoteSensorID(1))
// .with(new MotorOutputConfigs().withInverted(
// InvertedValue.Clockwise_Positive))),
// new Swerve.Module("FrontRight", new Translation2d(5.875, -10.875),
// new FX(3).with(drive),
// new FX(4).with(angle).with(encoder.withFeedbackRemoteSensorID(2))
// .with(new
// MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))),
// new Swerve.Module("RearLeft", new Translation2d(-10.875, 10.875),
// new FX(5).with(drive),
// new FX(6).with(angle).with(encoder.withFeedbackRemoteSensorID(3))
// .with(new MotorOutputConfigs().withInverted(
// InvertedValue.Clockwise_Positive))),
// new Swerve.Module("RearRight", new Translation2d(-10.875, -10.875),
// new FX(7).with(drive),
// new FX(8).with(angle).with(encoder.withFeedbackRemoteSensorID(4))
// .with(new
// MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))));
// }
// }
