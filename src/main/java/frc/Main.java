// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.system.Drivetrain;
import frc.system.mechanism.components.*;
import edu.wpi.first.wpilibj.TimedRobot;

public final class Main extends TimedRobot {
    /*
     * @Override
     * public Mechanism mechanism() {
     * Motor intakeMotors = new MotorSet(new Motor[] { new SRX(1), new SRX(2) });
     * 
     * SRX transitMotor = new SRX(3);
     * 
     * FX lPivotMotor = new FX(9);
     * FX rPivotMotor = new FX(10);
     * 
     * lPivotMotor.follow(rPivotMotor, true);
     * 
     * FX lShooterMotor = new FX(11);
     * FX rShooterMotor = new FX(12);
     * 
     * rShooterMotor.follow(lShooterMotor, false);
     * 
     * Intake intake = new Intake(intakeMotors, 0.6);
     * Transit transit = new Transit(transitMotor, 0.4, 0, 250);
     * Pivot pivot = new Pivot(lPivotMotor, 0.01, Rotation2d.fromDegrees(32), 0.42,
     * Rotation2d.fromDegrees(53));
     * Shooter shooter = new Shooter(lShooterMotor, 100.0, 1.0);
     * 
     * return new frc.system.mechanism.Mechanism(new Translation3d(1, 1, 1), intake,
     * transit, pivot, shooter);
     * }
     */
    public static Translation3d speakerPosition;

    Config conf = Config.get();

    private final CommandXboxController driver = new CommandXboxController(0);
    // private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final Drivetrain drivetrain = conf.drivetrain(subsystemsTable);

    private final Transit transit = new Transit(subsystemsTable, 5);
    private final Intake intake = new Intake(subsystemsTable, transit::hasNote);
    private final Shooter shooter = new Shooter(subsystemsTable, transit::hasNote);
    // private final Pivot pivot = new Pivot(null, kDefaultPeriod, null,
    // kDefaultPeriod, );

    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    @Override
    public void robotInit() {
        configButtonBindings();
        namedCommands();
    }

    private double MaxSpeed = 3.92;
    public double MaxAngularRate = Units.degreesToRadians(520);

    private void configButtonBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.drive(
                        -driver.getLeftY() * MaxSpeed,
                        -driver.getLeftX() * MaxSpeed,
                        -driver.getRightX() * MaxAngularRate));
        // pivot.setDefaultCommand(
        // pivot.toIntake());

        driver.rightTrigger().onTrue(
                // pivot.toIntake().andThen(
                intake.run().raceWith(transit.run()));// );
        driver.x().whileTrue(
                drivetrain.xState());
        driver.leftBumper().onTrue(
                drivetrain.zeroGyro());

        // operator.rightTrigger().onTrue(
        //         shooter.run().raceWith(shooter.waitPrimed().andThen(transit.run())));

        // operator.leftTrigger().onTrue(
        // shooter.shoot().alongWith(pivot.aim(0, 0)));// TODO: speeker pose

        // operator.a().onTrue(
        // drivetrain.DriveToThenPath(PathPlannerPath.fromPathFile("Amp"),
        // kDefaultPeriod)
        // .andThen(pivot.aim(0, 0))
        // .andThen(shooter.run().raceWith(shooter.waitPrimed().andThen(transit.run()))));//
        // TODO: amp pose

        // Pose2d robotPose = drivetrain.pose();

        // Translation3d origin = new Translation3d(robotPose.getX(),
        // robotPose.getY(),
        // 0);
        // Translation3d mechanism = origin.plus(originToMechanism);
        // Translation3d vector = Main.speakerPosition.minus(mechanism);

        // double distance = vector.getNorm();
        // double pitch = Math.asin(vector.getZ() / distance);
        // double yaw = Math.atan2(vector.getY(), vector.getX());

        // Translation2d xy = teleop.translation();

        // drivetrain.driveFacing(xy.getX(), xy.getY(), new Rotation2d(yaw));
        // pivot.aim(pitch, distance);
    }

    private void namedCommands() {
        // NamedCommands.registerCommand("intake",
        // pivot.toIntake().andThen(intake.run().raceWith(transit.run())));
        NamedCommands.registerCommand("shoot", shooter.run().raceWith(shooter.waitPrimed().andThen(transit.run())));

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
