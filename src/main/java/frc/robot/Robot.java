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
  private final DrivetrainReq drivetrain = new SwerveDrivetrain();
  private final DrivetrainSub drivetrainSub = new DrivetrainSub(drivetrain, null);

  private final IntakeRequirments intakeIOHardware = new IntakeHardware();
  private final IntakeSub intakeSub = new IntakeSub(intakeIOHardware);

  private final ArmRequirments armHardware = new ArmHardware();
  private final ArmSub armSub = new ArmSub(armHardware);

  private SendableChooser<Command> autoChooser;

  @Override
  public void robotInit() {
    // Logging
    Logger.recordMetadata("ProjectName", "MyProject");

    Logger.addDataReceiver(new NT4Publisher());
    Logger.disableDeterministicTimestamps();
    Logger.start();

    // Controlers
    SamKeyboard controler = new SamKeyboard(0);
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
    NamedCommands.registerCommand("IntakePose", armSub.IntakePositionAuton());
    NamedCommands.registerCommand("OutakePose", armSub.OutakeTopPositonAuton());
    NamedCommands.registerCommand("OutakeMidPose", armSub.OutakeMidPositonAuton());
    NamedCommands.registerCommand("Safe", armSub.SafePositon());
  }

  // this part should be the state machin
  public void configerButtonBindings() {

    cRobotButtons.zeroSensors().whileTrue(intakeSub.intakePice());
    cRobotButtons.zeroSensors().onFalse(intakeSub.stopIntake());

    cOporator.intakePice().onTrue(armSub.IntakePosition());
    cOporator.OuttakeTopPice().onTrue(armSub.OutakeTopPositon());
    cOporator.OuttakeMidPice().onTrue(armSub.OutakeMidPositon());

    cOporator.intakePice().onFalse(armSub.SafePositon());
    cOporator.OuttakeMidPice().onFalse(armSub.SafePositon());
    cOporator.OuttakeTopPice().onFalse(armSub.SafePositon());

    cOporator.setPose().onTrue(drivetrainSub.goToPose(new Pose2d(5, 5, new Rotation2d(Math.PI))));
    cOporator.setPose().onFalse(new InstantCommand(() -> {
      CommandScheduler.getInstance().cancel(drivetrainSub.getCurrentCommand());
    }));

    drivetrainSub.setDefaultCommand(drivetrainSub.drive(cDriver));
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
