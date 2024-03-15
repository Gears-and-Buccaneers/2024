// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.config.SwerveConfig;
import frc.system.CtreSwerve;
import frc.system.Intake;
import frc.system.Pivot;
import frc.system.Shooter;
import frc.system.Transit;

public final class Main extends TimedRobot {
    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    public static Translation3d speakerPosition;

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final CtreSwerve drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable);
    private final Shooter shooter = new Shooter(subsystemsTable);
    private final Pivot pivot = new Pivot(subsystemsTable, drivetrain::getVectorToSpeaker);

    // ---------------------------- Commands ------------------
    private Command rumble;
    private Command intakeNote;
    private Command primeAmp;
    private Command primeSpeaker;
    private Command shootNote;

    private void configCommands() {
        // LEDS
        // Rumble
        rumble = new Command() {
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
        };

        rumble.withTimeout(.25);
        rumble.addRequirements();
        rumble.setName("Rumble");

        // Intake
        intakeNote = pivot.toIntake().andThen(transit.runForwards().alongWith(intake.run()));

        intakeNote.until(transit::hasNote);
        intakeNote.addRequirements(transit, pivot, intake);
        intakeNote.setName("intakeNote");

        // Amp
        primeAmp = pivot.toAmp().alongWith(shooter.shootAmp());
        primeAmp.addRequirements(pivot, shooter);
        primeAmp.setName("PrimeAmp");

        // Speaker
        primeSpeaker = pivot.toSpeaker()
                .alongWith(shooter.shootSpeaker())
                .alongWith(drivetrain.driveFacingSpeaker(
                        () -> {
                            return Math.copySign(driver.getLeftX() * driver.getLeftX(), driver.getLeftX());
                        },
                        () -> {
                            return Math.copySign(driver.getLeftY() * driver.getLeftY(), -driver.getLeftY());
                        }));
        primeSpeaker.addRequirements(pivot, shooter);
        primeSpeaker.setName("PrimeSpeaker");

        // Shoot
        shootNote = shooter.waitPrimed().alongWith(drivetrain.brake())
                .andThen(transit.runForwards()).until(() -> !transit.hasNote()).andThen(shooter.stop(), transit.stop());
        shootNote.addRequirements(transit, shooter); // TODO: woried about this line and the previus
        shootNote.setName("Shoot");
    }

    private void configButtonBindings() {
        // ----------- DEFAULT COMMAND -----------

        transit.hasNoteTrigger.onTrue(rumble);
        transit.hasNoteTrigger.onFalse(rumble);

        pivot.setDefaultCommand(pivot.toIntake());
        // pivot.setDefaultCommand(pivot.manual(operator::getLeftY));

        // ----------- DRIVER CONTROLS -----------

        drivetrain.setDefaultCommand(
                drivetrain.controllerDrive(
                        () -> {
                            return Math.copySign(driver.getLeftX() * driver.getLeftX(), driver.getLeftX());
                        },
                        () -> {
                            return Math.copySign(driver.getLeftY() * driver.getLeftY(), -driver.getLeftY());
                        },
                        () -> {
                            // return Math.copySign(driver.getRightX() * driver.getRightX(),-
                            // driver.getRightX());
                            // TODO: ask if driver wants turning squared aswell
                            return -driver.getRightX();
                        }));

        driver.leftBumper().onTrue(
                drivetrain.zeroGyro());
        driver.x().whileTrue(
                drivetrain.brake());
        driver.leftTrigger().whileTrue(intakeNote);

        // ---------- OPERATOR CONTROLS ----------

        // TODO: path to the amp
        // operator.leftBumper().whileTrue(pivot.toAmp().alongWith(
        // shooter.shootAmp()));

        // operator.leftTrigger().whileTrue(//pivot.toSpeaker().alongWith(
        // //drivetrain.driveFacingSpeaker(driver::getLeftX, driver::getLeftY),
        // shooter.shootSpeaker());
        // operator.leftBumper().whileTrue(pivot.toIntake());
        // // operator.b().whileTrue(pivot.goToRadCmd(0.25));

        // operator.y().whileTrue(pivot.toSpeaker());

        // operator.a().onTrue(new InstantCommand(pivot::configPID));
        // operator.leftTrigger().whileTrue(shooter.shootSpeaker());
        // operator.start().onTrue(new InstantCommand(pivot::atIntake)); // zeros piviot
        boolean driveToAmp = false;

        operator.rightBumper().and(operator.leftBumper().or(operator.a())).whileTrue(shootNote);
        operator.leftBumper().whileTrue(primeSpeaker);
        operator.a().whileTrue(// TODO: Chechk this is the right button binding
                Commands.either(
                        drivetrain.DriveToThenPath(PathPlannerPath.fromPathFile("ScoreAmp")), // Score amp drives to the
                                                                                              // amp, this calles prime
                                                                                              // amp and shoot
                        primeAmp,
                        () -> driveToAmp));

        operator.x().whileTrue(intake.reverse());

        drivetrain.zeroGyro();

        // TODO: add climb command
    }

    private void configAutos() {
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
                        shooter.waitPrimed().andThen(transit.runForwards()))
                        .until(() -> !transit.hasNote())
                        .andThen(shooter.stop().alongWith(pivot.toIntake())));
        autonomousChooser.addOption("Shoot after 5sec",
                new WaitCommand(5).andThen(primeSpeaker.andThen(shootNote)));

        SmartDashboard.putData("auto", autonomousChooser);
    }

    private void configNamedCommands() {
        NamedCommands.registerCommand("Intake", intakeNote);
        NamedCommands.registerCommand("Shoot", shootNote);
        NamedCommands.registerCommand("PrimeAmp", primeAmp);
        NamedCommands.registerCommand("PrimeSpeaker", primeSpeaker);
    }

    // ---------------------------------------------------

    @Override
    public void robotInit() {
        // ------------------- Logging -------------------
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        configCommands();
        configAutos();
        configButtonBindings();
        configNamedCommands();
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
        // TODO: run full system check so its easy to do pre and post maches
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
