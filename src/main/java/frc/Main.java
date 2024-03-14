// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.config.SwerveConfig;
import frc.system.CtreSwerve;
import frc.system.Intake;
import frc.system.Pivot;
import frc.system.Shooter;
import frc.system.Transit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Main extends TimedRobot {
    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    Command autonomousCommand;
    SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    public static Translation3d speakerPosition;

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final CtreSwerve drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable, transit::hasNote);
    private final Shooter shooter = new Shooter(subsystemsTable, transit::hasNote);
    private final Pivot pivot = new Pivot(subsystemsTable, drivetrain::getVectorToSpeaker);

    private void namedCommands() {
        // NamedCommands.registerCommand("Intake", intake());
        // NamedCommands.registerCommand("Shoot", shoot());
        // NamedCommands.registerCommand("PrimeAmp", primeAmp());
        // NamedCommands.registerCommand("PrimeSpeaker", primeSpeaker());
    }

    static class TwoNoteState {
        double elapsed = 0;
        Timer timer = new Timer();
    }

    @Override
    public void robotInit() {
        // Righthanded origain is scorce side of blue allaince + x tords red TODO:
        // askiart

        // ------------------- Const -------------------
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(0.24, 2.66, 2.06) // TODO: chech numbers
                // Blue speaker
                : new Translation3d(16.31, 2.66, 2.06); // TODO: chech numbers

        // ------------------ Auto ------------------
        // adds pathplaner paths
        autonomousChooser = drivetrain.getAutoPaths();

        autonomousChooser.setDefaultOption("Nothing",
                new Command() {
                });

        autonomousChooser.addOption("Go go go go",
                drivetrain.controllerDrive(() -> -0.5, () -> 0, () -> 0)); // TODO: chech speed of back out

        autonomousChooser.addOption("Shoot",
                pivot.toSpeaker().alongWith(
                        shooter.shootSpeaker(),
                        shooter.waitPrimed().andThen(transit.run(true)))
                        .until(() -> !transit.hasNote())
                        .andThen(shooter.stop().alongWith(pivot.toIntake())));
        autonomousChooser.addOption("Shoot after 5sec",
                new WaitCommand(5).andThen(pivot.toSpeaker().alongWith(
                        shooter.shootSpeaker(),
                        shooter.waitPrimed().andThen(transit.run(true)))
                        .until(() -> !transit.hasNote())
                        .andThen(shooter.stop().alongWith(pivot.toIntake()))));

        TwoNoteState state = new TwoNoteState();

        autonomousChooser.addOption("TwoNote",
                // Shoot first note
                pivot.toSpeaker().alongWith(
                        shooter.shootSpeaker(),
                        shooter.waitPrimed().andThen(transit.run(true)))
                        // First note is shot
                        .until(() -> !transit.hasNote())
                        .andThen(
                                // Move arm back to intake and stop shooter
                                pivot.toIntake().alongWith(shooter.stop()).withTimeout(1),
                                transit.runSlow()
                                        .alongWith(intake.run(),
                                                drivetrain.controllerDrive(() -> 0, () -> -0.4, () -> 0),
                                                new InstantCommand(state.timer::restart))
                                        .until(transit::hasNote),
                                new InstantCommand(() -> {
                                    state.elapsed = state.timer.get();
                                    state.timer.restart();
                                }),
                                drivetrain.controllerDrive(() -> 0, () -> 0.4, () -> 0)
                                        .until(() -> state.timer.hasElapsed(state.elapsed)),
                                pivot.toSpeaker().alongWith(
                                        shooter.shootSpeaker(),
                                        shooter.waitPrimed().andThen(transit.run(true)))
                                        .until(() -> !transit.hasNote()),
                                shooter.stop().alongWith(pivot.toIntake())));

        SmartDashboard.putData("auto", autonomousChooser);

        Command rumble = new Command() {
            @Override
            public void initialize() {
                driver.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                operator.getHID().setRumble(RumbleType.kBothRumble, 0.5);
            }

            @Override
            public void end(boolean interrupted) {
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                operator.getHID().setRumble(RumbleType.kBothRumble, 0);
            }
        }.withTimeout(0.25);

        // ----------- DEFAULT COMMAND -----------

        transit.hasNoteTrigger.onTrue(rumble);
        transit.hasNoteTrigger.onFalse(rumble);

        // pivot.setDefaultCommand(pivot.toIntake());
        pivot.setDefaultCommand(pivot.manual(operator::getLeftY));

        // ----------- DRIVER CONTROLS -----------

        drivetrain.setDefaultCommand(
                drivetrain.controllerDrive(driver::getLeftX, driver::getLeftY, driver::getRightX));

        driver.leftBumper().onTrue(
                drivetrain.zeroGyro());
        driver.x().whileTrue(
                drivetrain.brake());
        driver.leftTrigger().whileTrue(
                transit.run(true).alongWith(intake.run()).until(transit::hasNote));

        // ---------- OPERATOR CONTROLS ----------

        // TODO: path to the amp
        // operator.leftBumper().whileTrue(pivot.toAmp().alongWith(
        // shooter.shootAmp()));

        // operator.leftTrigger().whileTrue(//pivot.toSpeaker().alongWith(
        // //drivetrain.driveFacingSpeaker(driver::getLeftX, driver::getLeftY),
        // shooter.shootSpeaker());

        operator.rightTrigger().whileTrue(transit.run(true));
        operator.rightBumper().whileTrue(transit.run(false));
        operator.leftBumper().whileTrue(pivot.toIntake());
        operator.b().whileTrue(pivot.goToRadCmd(0.25));

        operator.y().whileTrue(pivot.toSpeaker());

        operator.a().onTrue(new InstantCommand(pivot::configPID));
        operator.leftTrigger().whileTrue(shooter.shootSpeaker());
        operator.start().onTrue(new InstantCommand(pivot::atIntake)); // zeros piviot

        operator.x().whileTrue(intake.reverse());

        drivetrain.zeroGyro();

        // TODO: add climb command

        namedCommands();
    }

    private Command primeAmp() {
        return null;
    }

    private Command primeSpeaker() {
        return null;
    }

    private Command shoot() {
        return null;
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
        // if (!transit.hasNote())
        // runIntake.schedule();
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
