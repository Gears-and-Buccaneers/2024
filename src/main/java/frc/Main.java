// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.config.SwerveConfig;
import frc.system.*;
import frc.system.Intake.Intake;
import frc.system.Pivot.Pivot;
import frc.system.Shooter.Shooter;
import frc.system.Transit.Transit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Main extends TimedRobot {
    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    public static Translation3d speakerPosition;

    // Controllers
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final Drivetrain drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable, transit::hasNote);
    private final Shooter shooter = new Shooter(subsystemsTable, transit::hasNote);
    private final Pivot pivot = new Pivot(subsystemsTable);

    Main() {
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(0.24, 2.66, 2.06)
                // Blue speaker
                : new Translation3d(16.31, 2.66, 2.06);

        // Default Commands
        drivetrain.setDefaultCommand(
                drivetrain.drive(driver::getLeftX, driver::getLeftY, driver::getRightX));
        shooter.setDefaultCommand(
                shooter.run());
        pivot.setDefaultCommand(
                pivot.toIntake());

        // Driver
        driver.leftTrigger().whileTrue(intake()); // Option 1

        // Operator
        // Shoot
        operator.rightTrigger().whileTrue( // Option 2
                shooter.waitPrimed()// pivit at setpoint
                        .andThen(transit.shoot()));

        // Prime Amp
        // operator.leftBumper().whileTrue(
        // drivetrain.DriveToThenPath(PathPlannerPath.fromPathFile("amp")));
        // path finds to amp then scores in amp /\
        operator.leftBumper().whileTrue(
                pivot.toAmp().alongWith(shooter.shootAmp()));

        // Prime Speaker
        operator.leftTrigger().whileTrue(
                drivetrain.driveFacing(
                        driver::getLeftX,
                        driver::getLeftY,
                        null));// TODO: facing speeker
        operator.leftTrigger().whileTrue(
                pivot.toSpeaker(1, 1)
                        .alongWith(shooter.shootSpeaker()));

        // TODO: add climb command

        namedCommands();
    }

    // Commands
    private Command intake() {
        return intake.intake().raceWith(transit.intake());
    }

    private void namedCommands() {
        NamedCommands.registerCommand("Intake",
                intake());
        NamedCommands.registerCommand("Shoot",
                shooter.waitPrimed().andThen(transit.shoot()));
        NamedCommands.registerCommand("PrimeAmp",
                pivot.toAmp().alongWith(shooter.shootAmp()));
        NamedCommands.registerCommand("PrimeSpeaker",
                pivot.toSpeaker(1, 1).alongWith(shooter.shootSpeaker()));
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
