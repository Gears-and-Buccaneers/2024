// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.config.SwerveConfig;
import frc.system.CtreSwerve;
import frc.system.Intake;
import frc.system.Pivot;
import frc.system.Shooter;
import frc.system.Transit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private final CtreSwerve drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable, transit::hasNote);
    private final Shooter shooter = new Shooter(subsystemsTable, transit::hasNote);
    private final Pivot pivot = new Pivot(subsystemsTable, drivetrain::getVectorToSpeaker);

    Main() {
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(0.24, 2.66, 2.06)
                // Blue speaker
                : new Translation3d(16.31, 2.66, 2.06);

    }

    private void namedCommands() {
        // NamedCommands.registerCommand("Intake", intake());
        // NamedCommands.registerCommand("Shoot", shoot());
        // NamedCommands.registerCommand("PrimeAmp", primeAmp());
        // NamedCommands.registerCommand("PrimeSpeaker", primeSpeaker());
    }

    @Override
    public void robotInit() {
        SmartDashboard.putData("auto", drivetrain.getAutoPaths());
        // Default Commands
        drivetrain.setDefaultCommand(
                drivetrain.drive(driver::getLeftX, driver::getLeftY, driver::getRightX));

        // Driver
        driver.leftTrigger().whileTrue(
                intake.intake().raceWith(transit.intake())); // Option 1

        driver.x().whileTrue(drivetrain.brake());
        // Operator
        // Shoot
        operator.rightTrigger().whileTrue(
                shooter.shootSpeaker().alongWith(shooter.waitPrimed().andThen(transit.shoot())));

        // Prime Amp

        // operator.leftBumper().whileTrue(
        // drivetrain.DriveToThenPath(PathPlannerPath.fromPathFile("amp")));
        // path finds to amp then scores in amp /\
        operator.leftBumper().whileTrue(
                pivot.toAmp().alongWith(shooter.shootAmp()));

        // Prime Speaker
        operator.a().onTrue(pivot.congigPID());
        operator.x().whileTrue(pivot.manual(operator::getLeftY));
        operator.y().whileTrue(pivot.manual2(operator::getLeftY));
        operator.b().whileTrue(shooter.shootSpeaker());

        operator.leftTrigger().whileTrue(pivot.toSpeaker().raceWith(
                drivetrain.driveFacingSpeaker(driver::getLeftX, driver::getLeftY),
                shooter.shootSpeaker()));

        operator.start().onTrue(new InstantCommand(pivot::atIntake));

        // TODO: add climb command

        namedCommands();
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
