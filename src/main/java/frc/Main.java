// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.hardware.controller.Xbox;
import frc.system.Drivetrain;
import frc.system.Mechanism;
import frc.system.mechanism.components.Intake;
import frc.system.mechanism.components.Shooter;
import frc.system.mechanism.components.Transit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Main extends TimedRobot {
    public static Translation3d speakerPosition;

    Config conf = Config.get();

    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);

    final Mechanism mechanism = conf.mechanism();

    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final Drivetrain drivetrain = conf.drivetrain(subsystemsTable);

    private final Transit transit = new Transit(subsystemsTable, 5);
    private final Intake intake = new Intake(subsystemsTable, transit::hasNote);
    private final Shooter shooter = new Shooter(subsystemsTable, transit::hasNote);

    Main() {
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(0.24, 2.66, 2.06)
                // Blue speaker
                : new Translation3d(16.31, 2.66, 2.06);

        // TeleOp teleop = new TeleOp(drivetrain, 1, 1, driver.lX, driver.lY,
        // driver.rX);
        // drivetrain.setDefaultCommand(teleop);

        // Command brake = drivetrain.brake();

        Command intake = mechanism.intake();
        Command amp = mechanism.amp();
        // Command speaker = mechanism.speaker(drivetrain, teleop::translation);
        Command shoot = mechanism.shoot();

        operator.x().whileTrue(intake);
        // driver.x.whileTrue(brake);

        operator.a().whileTrue(shoot);
        // operator.lT.event(0.5).whileTrue(speaker);
        operator.leftBumper().whileTrue(amp);
        operator.start().onTrue(new InstantCommand(mechanism::tmp_zeroPivot));

        operator.b().whileTrue(mechanism.run(() -> mechanism.tmp_runPivot(operator.getLeftX())));

        // TODO: add climb command

        Translation2d stop = new Translation2d();

        NamedCommands.registerCommand("Intake", intake);
        NamedCommands.registerCommand("Amp", amp);
        // NamedCommands.registerCommand("Speaker", mechanism.speaker(drivetrain, () ->
        // stop));
        NamedCommands.registerCommand("Shoot", shoot);
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
