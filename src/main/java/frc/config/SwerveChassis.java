package frc.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import frc.Config;
import frc.impl.hardware.imu.Pigeon;
import frc.impl.hardware.motor.FX;
import frc.impl.system.drivetrain.Swerve;
import frc.system.Drivetrain;

public class SwerveChassis implements Config {
	@Override
	public Drivetrain drivetrain() {
		TalonFXConfiguration angleConf = new TalonFXConfiguration();
		TalonFXConfiguration driveConf = new TalonFXConfiguration();

		return new Swerve(new Pigeon(0),
				new Swerve.Module(new Translation2d(), new FX(angleConf, 1), new FX(driveConf, 5)),
				new Swerve.Module(new Translation2d(), new FX(angleConf, 2), new FX(driveConf, 6)),
				new Swerve.Module(new Translation2d(), new FX(angleConf, 3), new FX(driveConf, 7)),
				new Swerve.Module(new Translation2d(), new FX(angleConf, 4), new FX(driveConf, 8)));
	}
}
