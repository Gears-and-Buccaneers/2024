// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.AutoBuilder;
// pathplanner
import edu.wpi.first.net.PortForwarder;
// Logging
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.config.SwerveConfig;
import frc.system.*;

public class Robot extends TimedRobot {
	public static boolean isRedAlliance = false;

	SendableChooser<Command> autonomousChooser = new SendableChooser<>();

	// Controllers
	private final CommandXboxController driver = new CommandXboxController(0);

	// Subsystems
	private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

	private final Swerve drivetrain = SwerveConfig.swerve;

	private final Intake intake = new Intake(subsystemsTable);

	// Utilities
	static double squareInput(double input) {
		return Math.copySign(input * input, input);
	}

	private void configButtonBindings() {
		// ----------- DEFAULT COMMANDS -----------

		// NOTE: it appears that default commands are immediately rescheduled if they
		// finish. Looks like we'll have to implement some special logic to go to the
		// intake by default.
		// pivot.setDefaultCommand(pivot.toIntake());

		// ----------- DRIVER CONTROLS -----------

		drivetrain.setDefaultCommand(drivetrain.driveDutyCycle(
				isRedAlliance ? () -> squareInput(driver.getLeftY()) : () -> squareInput(-driver.getLeftY()),
				isRedAlliance ? () -> squareInput(driver.getLeftX()) : () -> squareInput(-driver.getLeftX()),
				// Note: rotation does not need to be inverted based on alliance side, since
				// rotational direction is not orientation-dependent.
				() -> -driver.getRightX()));

		driver.rightTrigger().whileTrue(intake.runOut());

		driver.leftBumper().onTrue(drivetrain.zeroGyro());

		// driver.rightBumper().onTrue(drivetrain.zeroGyroToSubwoofer());

		driver.x().whileTrue(drivetrain.brake());
	}

	private void configAutos() {
		// ------------------ AUTOS ------------------
		autonomousChooser.addOption("Drive 3m (Red)",
				drivetrain.driveVelocity(() -> -1, () -> 0, () -> 0)
						.until(() -> {
							return drivetrain.pose().getX() < -3.0;
						}).andThen(drivetrain.brake()));

		SmartDashboard.putData("auto", autonomousChooser);
	}

	@Override
	public void robotInit() {
		// Register alliance
		isRedAlliance = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent();
		// Add to NetworkTables for verification
		SmartDashboard.putBoolean("isRedAlliance", isRedAlliance);
		configButtonBindings();

		// ------------------- Logging -------------------
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}

		PortForwarder.add(5800, "photonvision.local", 5800);
		autonomousChooser = AutoBuilder.buildAutoChooser();
		configAutos();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		// if (drivetrain.isCamConnected()) {
		drivetrain.addPhotonVision();
		// }

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

	private Command autonomousCommand;

	@Override
	public void autonomousInit() {
		autonomousCommand = autonomousChooser.getSelected();
		if (autonomousCommand != null)
			autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
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
