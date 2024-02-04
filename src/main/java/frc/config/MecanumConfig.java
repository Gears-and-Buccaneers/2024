package frc.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import frc.Config;
import frc.hardware.imu.Pigeon;
import frc.hardware.profiledmotor.FX;
import frc.system.Drivetrain;
import frc.system.drivetrain.Mecanum;

public class MecanumConfig implements Config {
	@Override
	public Drivetrain drivetrain() {
		TalonFXConfiguration conf = new TalonFXConfiguration();

		return new Mecanum(new Pigeon(1),
				new Mecanum.Module(new Translation2d(), new FX(conf, 1)),
				new Mecanum.Module(new Translation2d(), new FX(conf, 2)),
				new Mecanum.Module(new Translation2d(), new FX(conf, 3)),
				new Mecanum.Module(new Translation2d(), new FX(conf, 4)));
	}
}
