// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.config.SwerveConfig;
import frc.system.*;

public class Robot extends TimedRobot {
	// Controllers
	private final CommandXboxController driver = new CommandXboxController(0);

	// Subsystems
	private final Swerve drivetrain = SwerveConfig.swerve;
	private final Intake intake = new Intake();

	private final Shooter shooterLeft = new Shooter(15);
	private final Shooter shooterRight = new Shooter(16);

	private final Vision vision = new Vision(new Vision.Target[] {
		new Vision.Target(0.2, 0.2, 0.3, 0.3, 200, shooterLeft.runHiInterrupt()),
		new Vision.Target(0.5, 0.5, 0.3, 0.3, 200, shooterRight.runHiInterrupt())
	});

	// Utilities
	static double squareInput(double input) {
		return Math.copySign(input * input, input);
	}

	@Override
	public void robotInit() {
		// Configure our controller bindings.

		// Driving
		drivetrain.setDefaultCommand(drivetrain.driveDutyCycle(
			() -> squareInput(driver.getLeftY()),
			() -> squareInput(driver.getLeftX()),
			// Note: rotation does not need to be inverted based on alliance side, since
			// rotational direction is not orientation-dependent.
			() -> -driver.getRightX()));
		driver.start().onTrue(drivetrain.zeroGyro());
		driver.x().whileTrue(drivetrain.brake());

		// Intake
		driver.leftTrigger().whileTrue(intake.runIn());	
		driver.a().whileTrue(intake.runOut());

		// Shooter
		driver.rightTrigger().whileTrue(new ParallelCommandGroup(
			shooterLeft.runLo(),
			shooterRight.runLo()
		));
		driver.rightBumper().whileTrue(new ParallelCommandGroup(
			shooterLeft.runHi(),
			shooterRight.runHi()
		));

		Command autoShooter = new Command() {
			@Override
			public void initialize() {
				// Resume the vision thread.
				vision.notifyStart();
			}

			@Override
			public void end(boolean interrupted) {
				// Pause the vision thread.
				vision.notifyStop();
			}
		}.alongWith(shooterLeft.runLo(), shooterRight.runLo());

		driver.leftBumper().whileTrue(autoShooter);
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
