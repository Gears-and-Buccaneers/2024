package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Arm.*;
import frc.robot.Subsystems.Drivetrain.*;
import frc.robot.Subsystems.Intake.*;
import frc.robot.joysticks.*;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// do some copy and paste from this
// https://github.com/FRCTeam2910/2023CompetitionRobot-Public/tree/main
public class Robot extends LoggedRobot {
  // Controllers
  private Driver cDriver;
  private Operator cOperator;
  private RobotButtons cRobotButtons;

  // subsystems
  private final SwerveDrivetrain drivetrainIO = new SwerveDrivetrain();
  private final DrivetrainSub drivetrain = new DrivetrainSub(drivetrainIO, drivetrainIO);

  private final IntakeRequirements intakeIO = new IntakeHardware();
  private final IntakeSub intake = new IntakeSub(intakeIO);

  private final ArmRequirements armIO = new ArmHardware();
  private final ArmSub arm = new ArmSub(armIO);

  private SendableChooser<Command> autoChooser;

  @Override
  public void robotInit() {
    // Logging
    Logger.recordMetadata("ProjectName", "MyProject");

    Logger.addDataReceiver(new NT4Publisher());
    Logger.disableDeterministicTimestamps();
    Logger.start();

    // Controllers
    Xbox controller = new Xbox(0);
    cRobotButtons = new RealRobotButtons();
    cDriver = controller;
    cOperator = controller;

    // Button Bindings
    configerButtonBindings();

    nameCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }

  public void nameCommands() {
    NamedCommands.registerCommand("IntakePose", arm.IntakePositionAuton());
    NamedCommands.registerCommand("OuttakePose", arm.OuttakeTopPositonAuton());
    NamedCommands.registerCommand("OuttakeMidPose", arm.OuttakeMidPositonAuton());
    NamedCommands.registerCommand("Safe", arm.SafePositon());
  }

  // this part should be the state machine
  public void configerButtonBindings() {
    drivetrain.setDefaultCommand(drivetrain.drive(cDriver));

    cRobotButtons.zeroSensors().whileTrue(intake.intakePiece());
    cRobotButtons.zeroSensors().onFalse(intake.stopIntake());

    cOperator.intakePiece().onTrue(arm.IntakePosition());
    cOperator.outtakeTopPiece().onTrue(arm.OuttakeTopPositon());
    cOperator.outtakeMidPiece().onTrue(arm.OuttakeMidPositon());

    cOperator.intakePiece().onFalse(arm.SafePositon());
    cOperator.outtakeMidPiece().onFalse(arm.SafePositon());
    cOperator.outtakeTopPiece().onFalse(arm.SafePositon());

    cOperator.setPose().onTrue(drivetrain.goToPose(new Pose2d(5, 5, new Rotation2d(Math.PI))));
    cOperator
        .setPose()
        .onFalse(
            new InstantCommand(
                () -> {
                  CommandScheduler.getInstance().cancel(drivetrain.getCurrentCommand());
                }));

    cOperator.intakePiece().onTrue(cDriver.rumble());
    cOperator
        .intakePiece()
        .onFalse(
            new InstantCommand(
                () -> {
                  cDriver.rumble().cancel();
                }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
