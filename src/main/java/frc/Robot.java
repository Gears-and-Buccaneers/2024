// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.AutoBuilder;
// pathplanner
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.PortForwarder;
// Logging
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.config.SwerveConfig;
import frc.system.*;
import frc.cmd.*;

public class Robot extends TimedRobot {

    SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final Swerve drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable);
    private final Shooter shooter = new Shooter(subsystemsTable);
    private final Pivot pivot = new Pivot(subsystemsTable);

    // Commands

    // Utilities
    static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    public Robot() {
        // ------------------- Logging -------------------
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // TODO: re-enable vision once the jitter is solved.
        // new Nt().register(drivetrain);
        PortForwarder.add(5800, "photonvision.local", 5800);
        configNamedCommands();
        autonomousChooser = AutoBuilder.buildAutoChooser();
        configAutos();

    }

    // TODO: add comments
    private class Commands {
        Command rumble(RumbleType type, double strength, double duration,
                CommandXboxController... controllers) {
            return new Command() {
                @Override
                public void initialize() {
                    for (CommandXboxController controller : controllers)
                        controller.getHID().setRumble(type, strength);
                }

                @Override
                public void end(boolean interrupted) {
                    for (CommandXboxController controller : controllers)
                        controller.getHID().setRumble(type, 0);
                }
            }.withTimeout(duration);
        }

        Command intakeNote() {
            return pivot.toIntake()
                    .alongWith(
                            intake.runIn().until(() -> transit.hasNote())
                                    .andThen(rumble(RumbleType.kBothRumble, .75, .5, driver, operator)),
                            transit.feedIn());
        }

        Command primeAmp() {
            return pivot.toAmp().alongWith(shooter.shootAmp());
        }

        Command primeSpeaker() {
            return new AimSpeaker(drivetrain, pivot, shooter).alongWith(
                    shooter.shootSpeaker(),
                    pivot.toSpeaker());
        }

        Command primeSubwoofer() {
            return pivot.toSubwoofer().alongWith(shooter.shootSpeaker());
        }

        Command waitThenFeed() {
            return new WaitCommand(2).andThen(transit.feedOut());
        }

        Command shoot() {
            return transit.feedOut().andThen(
                    rumble(RumbleType.kBothRumble, .75, .5, driver, operator),
                    pivot.toIntake());
        }

        // Auto Commands
        Command intakeNoteAuto() {
            return pivot.toIntake()
                    .alongWith(
                            intake.runIn(),
                            transit.runForwards())
                    .until(() -> transit.hasNote());
        }

        Command primeSpeakerAuto() {
            return new AimSpeaker(drivetrain, pivot, shooter).alongWith(
                    shooter.shootSpeaker(),
                    pivot.toSpeaker(),
                    drivetrain.driveDutyCycle(() -> 0, () -> 0, () -> 0));
        }

        Command shootSpeakerAuto() {
            return primeSpeakerAuto()
                    .raceWith(
                            new WaitUntilCommand(2)
                                    .andThen(transit.runForwards().until((() -> !transit.hasNote()))));
        }

        Command shootSpeakerAuto2() {
            return primeSpeakerAuto()
                    .raceWith(
                            new WaitUntilCommand(() -> drivetrain.isAimed() && pivot.isAimed())
                                    .andThen(transit.runForwards().until((() -> !transit.hasNote()))));
        }
    }

    private final Commands cmds = new Commands();

    private void configButtonBindings() {
        // ----------- DEFAULT COMMANDS -----------

        // NOTE: it appears that default commands are immediately rescheduled if they
        // finish. Looks like we'll have to implement some special logic to go to the
        // intake by default.
        // pivot.setDefaultCommand(pivot.toIntake());
        pivot.setDefaultCommand(pivot.dutyCycleCtrl(operator::getLeftY));

        // ----------- DRIVER CONTROLS -----------

        drivetrain.setDefaultCommand(drivetrain.driveDutyCycle(
                () -> squareInput(driver.getLeftY()),
                () -> squareInput(driver.getLeftX()),
                // TODO: ask if driver wants turning squared as well
                () -> -driver.getRightX()));

        driver.leftBumper().onTrue(drivetrain.zeroGyro());
        driver.rightBumper().onTrue(drivetrain.zeroGyroToSubwoofer());
        driver.x().whileTrue(drivetrain.brake());

        driver.leftTrigger().onTrue(cmds.intakeNote());

        driver.y().onTrue(cmds.shootSpeakerAuto2());
        // ---------- OPERATOR CONTROLS ----------

        // TODO: Pathfind to the amp using a PathfindToPose command
        operator.leftBumper().whileTrue(cmds.primeAmp());
        operator.leftTrigger().whileTrue(cmds.primeSpeaker());

        operator.rightTrigger().whileTrue(cmds.shoot());
        operator.rightBumper().whileTrue(transit.runForwards());

        operator.a().onTrue(transit.feedIn());
        operator.y().whileTrue(cmds.primeSubwoofer());

        operator.b().whileTrue(transit.runBackward());
        operator.x().whileTrue(intake.runOut());

        // Zeroes the pivot, assuming it is at intaking position.
        operator.start().onTrue(new InstantCommand(pivot::currentZeroingSequence));

        // TODO: Add climb command
    }

    private void configAutos() {
        configNamedCommands();
        // ------------------ AUTOS ------------------

        autonomousChooser.addOption("Shoot",
                cmds.primeSpeaker().raceWith(
                        new WaitUntilCommand(() -> drivetrain.isAimed() && pivot.isAimed())
                                .andThen(cmds.waitThenFeed()))
                        // TODO: configure the next two as default commands (not working)
                        .andThen(shooter.stop().alongWith(pivot.toIntake())));

        autonomousChooser.addOption("Shoot against subwoofer",
                new WaitCommand(5).andThen(cmds.primeSubwoofer().raceWith(cmds.waitThenFeed())));

        autonomousChooser.addOption("Drive 3m (Red)",
                drivetrain.driveVelocity(() -> -1, () -> 0, () -> 0)
                        .until(() -> {
                            return drivetrain.pose().getX() < -3.0;
                        }).andThen(drivetrain.brake()));

        SmartDashboard.putData("auto", autonomousChooser);
    }

    private void configNamedCommands() {
        NamedCommands.registerCommand("intake", cmds.intakeNoteAuto());
        NamedCommands.registerCommand("feedIn", transit.feedIn());
        NamedCommands.registerCommand("shoot", cmds.shootSpeakerAuto());
    }

    @Override
    public void robotInit() {

        configButtonBindings();
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
        // TODO: Run an automated full systems check
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
