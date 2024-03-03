// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.command.TeleOp;
import frc.hardware.controller.Xbox;
import frc.system.Drivetrain;
import frc.system.Mechanism;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Main extends TimedRobot {
	public static Translation3d speakerPosition;

	Config conf = Config.get();

	Xbox driver = new Xbox(0);
	Xbox operator = new Xbox(1);

	final Drivetrain drivetrain = conf.drivetrain();
	final Mechanism mechanism = conf.mechanism();

	public static void main(String... args) {
		RobotBase.startRobot(Main::new);
	}

	Main() {
		speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
				?
				// Red speaker
				new Translation3d(0.24, 2.66, 2.06)
				// Blue speaker
				: new Translation3d(16.31, 2.66, 2.06);

		TeleOp teleop = new TeleOp(drivetrain, 1, 1, driver.lX, driver.lY, driver.rX);
		drivetrain.setDefaultCommand(teleop);

		Command brake = drivetrain.brake();

		Command intake = mechanism.intake();
		Command amp = mechanism.amp();
		Command speaker = mechanism.speaker(drivetrain, teleop::translation);
		Command shoot = mechanism.shoot();

		driver.rT.event(0.5).whileTrue(intake);
		driver.x.whileTrue(brake);

		operator.rT.event(0.5).whileTrue(shoot);
		operator.lT.event(0.5).whileTrue(speaker);
		operator.lB.whileTrue(amp);
		operator.start.onTrue(new InstantCommand(mechanism::tmp_zeroPivot));

		operator.b.whileTrue(mechanism.run(() -> mechanism.tmp_runPivot(operator.lY.get())));

		// TODO: add climb command

		Translation2d stop = new Translation2d();

		NamedCommands.registerCommand("Intake", intake);
		NamedCommands.registerCommand("Amp", amp);
		NamedCommands.registerCommand("Speaker", mechanism.speaker(drivetrain, () -> stop));
		NamedCommands.registerCommand("Shoot", shoot);
	}

	@Override
	public void robotInit() {
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
