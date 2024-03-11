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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Main extends TimedRobot {
    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

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

    static class TwoNoteState {
        double elapsed = 0;
        Timer timer = new Timer();
    }

    @Override
    public void robotInit() {
        double subwooferAngle = -0.0530455469;

        chooser.addOption("Nothing", new Command() {
        });

        chooser.addOption("Go go go go", drivetrain.drive(() -> -0.5, () -> 0, () -> 0));

        chooser.addOption("Shoot",
                pivot.goToCmd(subwooferAngle).alongWith(
                        shooter.shootSpeaker(),
                        shooter.waitPrimed().andThen(transit.run(true)))
                        .until(() -> !transit.hasNote())
                        .andThen(shooter.stop().alongWith(pivot.toIntake())));

        TwoNoteState state = new TwoNoteState();

        chooser.addOption("TwoNote",
                // Shoot first note
                pivot.goToCmd(subwooferAngle).alongWith(
                        shooter.shootSpeaker(),
                        shooter.waitPrimed().andThen(transit.run(true)))
                        // First note is shot
                        .until(() -> !transit.hasNote())
                        .andThen(
                                // Move arm back to intake and stop shooter
                                pivot.toIntake().alongWith(shooter.stop()).withTimeout(1),
                                transit.runSlow()
                                        .alongWith(intake.run(), drivetrain.drive(() -> 0, () -> -0.4, () -> 0),
                                                new InstantCommand(state.timer::restart))
                                        .until(transit::hasNote),
                                new InstantCommand(() -> {
                                    state.elapsed = state.timer.get();
                                    state.timer.restart();
                                }),
                                drivetrain.drive(() -> 0, () -> 0.4, () -> 0)
                                        .until(() -> state.timer.hasElapsed(state.elapsed)),
                                pivot.goToCmd(subwooferAngle).alongWith(
                                        shooter.shootSpeaker(),
                                        shooter.waitPrimed().andThen(transit.run(true)))
                                        .until(() -> !transit.hasNote()),
                                shooter.stop().alongWith(pivot.toIntake())));

        SmartDashboard.putData("auto", chooser);

        // Command rumble = new Command() {
        // @Override
        // public void initialize() {
        // driver.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        // operator.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        // }

        // @Override
        // public void end(boolean interrupted) {
        // driver.getHID().setRumble(RumbleType.kBothRumble, 0);
        // operator.getHID().setRumble(RumbleType.kBothRumble, 0);
        // }
        // }.withTimeout(0.25);

        // ----------- DEFAULT COMMAND -----------

        // transit.hasNoteTrigger.onTrue(rumble);
        // transit.hasNoteTrigger.onFalse(rumble);

        // ----------- DRIVER CONTROLS -----------

        drivetrain.setDefaultCommand(
                drivetrain.drive(driver::getLeftX, driver::getLeftY, driver::getRightX));
        driver.leftBumper().onTrue(drivetrain.zeroGyro());
        driver.x().whileTrue(drivetrain.brake());
        driver.leftTrigger().whileTrue(transit.run(true).alongWith(intake.run()).until(transit::hasNote));

        // pivot.setDefaultCommand(pivot.toIntake());

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

        operator.b().whileTrue(pivot.goToCmd(0.25));

        operator.y().whileTrue(pivot.goToCmd(subwooferAngle));
        pivot.setDefaultCommand(pivot.manual(operator::getLeftY));

        operator.a().onTrue(new InstantCommand(pivot::configPID));
        operator.leftTrigger().whileTrue(shooter.shootSpeaker());
        operator.start().onTrue(new InstantCommand(pivot::atIntake));

        operator.x().whileTrue(intake.reverse());

        drivetrain.zeroGyro();

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
        autonomousCommand = chooser.getSelected();
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
