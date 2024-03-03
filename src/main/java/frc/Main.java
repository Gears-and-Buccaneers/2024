// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.command.TeleOp;
import frc.hardware.Motor;
import frc.hardware.controller.Xbox;
import frc.hardware.motor.SRX;
import frc.system.Drivetrain;
import frc.system.Mechanism;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Main extends TimedRobot {
	public static Translation3d speakerPosition;

	Config conf = Config.get();

	public Motor back = new SRX(9);
	public Motor front = new SRX(20);

	Xbox driver = new Xbox(0);

	final Drivetrain drivetrain = conf.drivetrain();
	final Mechanism mechanism = conf.mechanism();
	final TeleOp teleop = new TeleOp(drivetrain, 1, 1, driver.lX, driver.lY, driver.rX);

	public static void main(String... args) {
		RobotBase.startRobot(Main::new);
	}

	Main() {
		Translation2d stop = new Translation2d();

		NamedCommands.registerCommand("Intake", mechanism.intake());
		NamedCommands.registerCommand("Amp", mechanism.amp());
		NamedCommands.registerCommand("Speaker", mechanism.speaker(drivetrain, () -> stop));
		NamedCommands.registerCommand("Shoot", mechanism.shoot());
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
		front.setPercent(0.2);
		back.setPercent(0.2);
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
		front.setPercent(0);
		front.setPercent(0);
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
