package frc.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

import frc.Config;
import frc.hardware.motor.SRX;
import frc.hardware.profiledmotor.FX;
import frc.system.Drivetrain;
import frc.system.Mechanism;
import frc.system.drivetrain.CtreSwerve;
import frc.system.mechanism.components.Intake;
import frc.system.mechanism.components.Pivot;
import frc.system.mechanism.components.Shooter;
import frc.system.mechanism.components.Transit;

public class CtreSwerveConfig implements Config {
	@Override
	public Drivetrain drivetrain() {

		// The steer motor uses any SwerveModule.SteerRequestType control request with
		// the
		// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
		Slot0Configs steerGains = new Slot0Configs()
				.withKP(100).withKI(0).withKD(0.2)
				.withKS(0).withKV(1.5).withKA(0);
		// When using closed-loop control, the drive motor uses the control
		// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
		Slot0Configs driveGains = new Slot0Configs()
				.withKP(3).withKI(0).withKD(0)
				.withKS(0).withKV(0).withKA(0);

		// The closed-loop output type to use for the steer motors;
		// This affects the PID/FF gains for the steer motors
		ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
		// The closed-loop output type to use for the drive motors;
		// This affects the PID/FF gains for the drive motors
		ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

		// The stator current at which the wheels start to slip;
		// This needs to be tuned to your individual robot
		double kSlipCurrentA = 300.0;

		// Theoretical free speed (m/s) at 12v applied output;
		// This needs to be tuned to your individual robot
		double kSpeedAt12VoltsMps = 3.92;

		// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
		// This may need to be tuned to your individual robot
		double kCoupleRatio = 3.5714285714285716;

		double kDriveGearRatio = 8.142857142857142;
		double kSteerGearRatio = 21.428571428571427;
		double kWheelRadiusInches = 2;

		boolean kSteerMotorReversed = true;
		boolean kInvertLeftSide = false;
		boolean kInvertRightSide = true;

		String kCANbusName = "";
		int kPigeonId = 1;

		// These are only used for simulation
		double kSteerInertia = 0.00001;
		double kDriveInertia = 0.001;
		// Simulated voltage necessary to overcome friction
		double kSteerFrictionVoltage = 0.25;
		double kDriveFrictionVoltage = 0.25;

		SwerveDrivetrainConstants constants = new SwerveDrivetrainConstants()
				.withPigeon2Id(kPigeonId)
				.withCANbusName(kCANbusName);

		SwerveModuleConstantsFactory kCreator = new SwerveModuleConstantsFactory()
				.withDriveMotorGearRatio(kDriveGearRatio)
				.withSteerMotorGearRatio(kSteerGearRatio)
				.withWheelRadius(kWheelRadiusInches)
				.withSlipCurrent(kSlipCurrentA)
				.withSteerMotorGains(steerGains)
				.withDriveMotorGains(driveGains)
				.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
				.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
				.withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withSteerFrictionVoltage(kSteerFrictionVoltage)
				.withDriveFrictionVoltage(kDriveFrictionVoltage)
				.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
				.withCouplingGearRatio(kCoupleRatio)
				.withSteerMotorInverted(kSteerMotorReversed);

		// Front Left
		int kFrontLeftDriveMotorId = 1;
		int kFrontLeftSteerMotorId = 2;
		int kFrontLeftEncoderId = 1;
		double kFrontLeftEncoderOffset = 0.43603515625;

		double kFrontLeftXPosInches = 8.375;
		double kFrontLeftYPosInches = 10.875;

		// Front Right
		int kFrontRightDriveMotorId = 3;
		int kFrontRightSteerMotorId = 4;
		int kFrontRightEncoderId = 2;
		double kFrontRightEncoderOffset = -0.05615234375;

		double kFrontRightXPosInches = 8.375;
		double kFrontRightYPosInches = -10.875;

		// Back Left
		int kBackLeftDriveMotorId = 5;
		int kBackLeftSteerMotorId = 6;
		int kBackLeftEncoderId = 3;
		double kBackLeftEncoderOffset = -0.01953125;

		double kBackLeftXPosInches = -8.375;
		double kBackLeftYPosInches = 10.875;

		// Back Right
		int kBackRightDriveMotorId = 7;
		int kBackRightSteerMotorId = 8;
		int kBackRightEncoderId = 4;
		double kBackRightEncoderOffset = 0.2880859375;

		double kBackRightXPosInches = -8.375;
		double kBackRightYPosInches = -10.875;

		SwerveModuleConstants frontLeft = kCreator.createModuleConstants(
				kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
				kFrontLeftEncoderOffset,
				Units.inchesToMeters(kFrontLeftXPosInches),
				Units.inchesToMeters(kFrontLeftYPosInches),
				kInvertLeftSide);
		SwerveModuleConstants frontRight = kCreator.createModuleConstants(
				kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
				kFrontRightEncoderOffset,
				Units.inchesToMeters(kFrontRightXPosInches),
				Units.inchesToMeters(kFrontRightYPosInches),
				kInvertRightSide);
		SwerveModuleConstants backLeft = kCreator.createModuleConstants(
				kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId,
				kBackLeftEncoderOffset,
				Units.inchesToMeters(kBackLeftXPosInches),
				Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
		SwerveModuleConstants backRight = kCreator.createModuleConstants(
				kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
				kBackRightEncoderOffset,
				Units.inchesToMeters(kBackRightXPosInches),
				Units.inchesToMeters(kBackRightYPosInches),
				kInvertRightSide);

		return new CtreSwerve(constants, frontLeft, frontRight, backLeft, backRight);
	}

	@Override
	public Mechanism mechanism() {
		SRX intakeMotor = new SRX(0);
		SRX transitMotor = new SRX(0);
		FX pivotMotor = new FX(0);
		FX shooterMotor = new FX(0);

		Intake intake = new Intake(intakeMotor, 0.6);
		Transit transit = new Transit(transitMotor, 0.4);
		Pivot pivot = new Pivot(pivotMotor, 0.01);
		Shooter shooter = new Shooter(shooterMotor, 100.0, 1.0);

		// TODO Auto-generated method stub
		return new frc.system.mechanism.Mechanism(intake, transit, pivot, shooter);
	}
}
