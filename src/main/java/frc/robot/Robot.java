package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsytems.Arm.*;
import frc.robot.Subsytems.Drivetrain.*;
import frc.robot.Subsytems.Intake.*;
import frc.robot.joystics.*;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// do some copy and past from this
// https://github.com/FRCTeam2910/2023CompetitionRobot-Public/tree/main
public class Robot extends LoggedRobot {
  // Controlers
  private Driver cDriver;
  private Oporator cOporator;
  private RobotButtons cRobotButtons;

  // subsytems
  private final SwerveDrivetrain drivetrainIO = new SwerveDrivetrain();
  private final DrivetrainSub drivetrain = new DrivetrainSub(drivetrainIO, drivetrainIO);

  private final IntakeRequirments intakeIO = new IntakeHardware();
  private final IntakeSub intake = new IntakeSub(intakeIO);

  private final ArmRequirments armIO = new ArmHardware();
  private final ArmSub arm = new ArmSub(armIO);

  private SendableChooser<Command> autoChooser;

  @Override
  public void robotInit() {
    // Logging
    Logger.recordMetadata("ProjectName", "MyProject");

    Logger.addDataReceiver(new NT4Publisher());
    Logger.disableDeterministicTimestamps();
    Logger.start();

    // Controlers
    Xbox controler = new Xbox(0);
    cRobotButtons = new RealRobotButtons();
    cDriver = controler;
    cOporator = controler;

    // Button Bindings
    configerButtonBindings();

    nameCommangs();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }

  public void nameCommangs() {
    NamedCommands.registerCommand("IntakePose", arm.IntakePositionAuton());
    NamedCommands.registerCommand("OutakePose", arm.OutakeTopPositonAuton());
    NamedCommands.registerCommand("OutakeMidPose", arm.OutakeMidPositonAuton());
    NamedCommands.registerCommand("Safe", arm.SafePositon());
  }

  // this part should be the state machin
  public void configerButtonBindings() {
    drivetrain.setDefaultCommand(drivetrain.drive(cDriver));

    cRobotButtons.zeroSensors().whileTrue(intake.intakePice());
    cRobotButtons.zeroSensors().onFalse(intake.stopIntake());

    cOporator.intakePice().onTrue(arm.IntakePosition());
    cOporator.OuttakeTopPice().onTrue(arm.OutakeTopPositon());
    cOporator.OuttakeMidPice().onTrue(arm.OutakeMidPositon());

    cOporator.intakePice().onFalse(arm.SafePositon());
    cOporator.OuttakeMidPice().onFalse(arm.SafePositon());
    cOporator.OuttakeTopPice().onFalse(arm.SafePositon());

    cOporator.setPose().onTrue(drivetrain.goToPose(new Pose2d(5, 5, new Rotation2d(Math.PI))));
    cOporator.setPose().onFalse(new InstantCommand(() -> {
      CommandScheduler.getInstance().cancel(drivetrain.getCurrentCommand());
    }));

    cOporator.intakePice().onTrue(cDriver.rumble());
    cOporator.intakePice().onFalse(new InstantCommand(() -> {
      cDriver.rumble().cancel();
    }));
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
    autoChooser.getSelected().schedule();
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
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
