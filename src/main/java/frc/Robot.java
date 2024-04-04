// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.AutoBuilder;
// pathplanner
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public static boolean isRedAlliance;

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
        // Register alliance
        isRedAlliance = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent();
        configButtonBindings();
        // Add to NetworkTables for verification
        SmartDashboard.putBoolean("isRedAlliance", isRedAlliance);

        // ------------------- Logging -------------------
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

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

        /**
         * <ol>
         * <li>Runs both the intake and the transit in.</li>
         * <li>the intake will stop once the note is in the transit and then it will
         * rumble the driver and operator</li>
         * <li>the transit will stop when it has finished its feed in procedure (runs
         * it in till it seas a note than backs it out and then runs it back in until
         * it sees the note)</li>
         * </ol>
         * 
         * @return Command that requires PIVOT, INTAKE and TRANSIT
         */
        Command intakeNote() {
            return pivot.toIntakeUntilAimed()
                    .alongWith(
                            intake.runIn().until(() -> transit.hasNote())
                                    .andThen(rumble(RumbleType.kBothRumble, .75, .5, driver, operator)),
                            transit.feedIn());
        }

        Command primeAmp() {
            return pivot.toAmp().alongWith(shooter.shootAmp(), new Command() {
                @Override
                public void initialize() {
                    // Point the shooter towards positive y, the direction of the amp entrance.
                    drivetrain.setRotationOverride(Rotation2d.fromDegrees(90));
                }

                @Override
                public void end(boolean interrupted) {
                    drivetrain.setRotationOverride(null);
                }
            });
        }

        Command primeSpeaker() {
            return new AimSpeaker(drivetrain, pivot, shooter);
        }

        Command primeSubwoofer() {
            return pivot.toSubwoofer().alongWith(shooter.shootSpeaker());
        }

        Command waitThenFeed() {
            return new WaitCommand(2).andThen(transit.feedOut());
        }

        /**
         * Currently crashes the robot code; do not use. 
         * TODO: Why?!
         * 
         * @return
         */
        @Deprecated
        Command shoot() {
            return transit.feedOut().andThen(
                    rumble(RumbleType.kBothRumble, .75, .5, driver, operator),
                    pivot.toIntakeUntilAimed());
        }

        // Auto Commands
        Command intakeNoteAuto() {
            return pivot.toIntakeUntilAimed()
                    .andThen(
                            intake.runIn().alongWith(transit.runForwards()))
                    .until(() -> transit.hasNote());
        }

        Command primeSpeakerAuto() {
            return primeSpeaker().alongWith(
                    drivetrain.driveDutyCycle(() -> 0, () -> 0, () -> 0));
        }

        Command shootSpeakerAuto() {
            return primeSpeakerAuto()
                    .alongWith(new WaitCommand(1).andThen(transit.runForwards()))
                    .until((() -> !transit.hasNote()));
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
    
        shooter.setDefaultCommand(shooter.VelocityOpenLoopCmd(true, 500));
        
        // ----------- DRIVER CONTROLS -----------
        
        drivetrain.setDefaultCommand(drivetrain.driveDutyCycle(
            isRedAlliance ? () -> squareInput(driver.getLeftY()) : () -> squareInput(-driver.getLeftY()),
            isRedAlliance ? () -> squareInput(driver.getLeftX()) : () -> squareInput(-driver.getLeftX()),
            // Note: rotation does not need to be inverted based on alliance side, since rotational direction is not orientation-dependent.
            () -> -driver.getRightX()));
            
        driver.leftTrigger().whileTrue(cmds.intakeNote());
        driver.leftBumper().onTrue(drivetrain.zeroGyro());
        
        // driver.rightBumper().onTrue(drivetrain.zeroGyroToSubwoofer());
        
        driver.x().whileTrue(drivetrain.brake());
        
        // ---------- OPERATOR CONTROLS ----------
            
        pivot.setDefaultCommand(pivot.dutyCycleCtrl(operator::getLeftY));

        // TODO: make button map printout for operator
        operator.leftTrigger().whileTrue(cmds.primeSpeaker());
        // TODO: Pathfind to the amp using a PathfindToPose command
        operator.leftBumper().whileTrue(cmds.primeAmp());

        // operator.rightTrigger().whileTrue(cmds.shoot());
        operator.rightTrigger().whileTrue(transit.runForwards());
        operator.rightBumper().whileTrue(transit.runBackward());

        operator.a().whileTrue(shooter.shootSpeaker());
        operator.b().whileTrue(pivot.toIntakeUntilAimed());
        operator.y().whileTrue(cmds.primeSubwoofer());
        operator.x().whileTrue(intake.runOut());

        // Zeroes the pivot, assuming it is at intake position.
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
                        .andThen(shooter.stop().alongWith(pivot.toIntakeUntilAimed())));

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
