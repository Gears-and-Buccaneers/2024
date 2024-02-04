// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.command.TeleOp;
import frc.hardware.controller.Xbox;
import frc.system.Drivetrain;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Main extends TimedRobot {
	Config conf = Config.get();

	Xbox driver = new Xbox(0);

	final Drivetrain drivetrain = conf.drivetrain();
	final TeleOp teleop = new TeleOp(drivetrain, 1, 1, driver.lX, driver.lY, driver.rX, driver.a);

	public static void main(String... args) {
		RobotBase.startRobot(Main::new);
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
