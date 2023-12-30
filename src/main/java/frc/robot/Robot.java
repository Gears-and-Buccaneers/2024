package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsytems.Arm.*;
import frc.robot.Subsytems.Drivetrain.DrivetrainRequirments;
import frc.robot.Subsytems.Drivetrain.DrivetrainSub;
import frc.robot.Subsytems.Drivetrain.DrivetrainSwerve;
import frc.robot.Subsytems.Intake.*;
import frc.robot.joystics.*;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// do some copy and past from this
// https://github.com/FRCTeam2910/2023CompetitionRobot-Public/tree/main
public class Robot extends LoggedRobot {
  // Controlers
  private Driver cDriver;
  private Oporator cOporator;
  private RobotButtons cRobotButtons;

  // subsytems
  private final DrivetrainRequirments drivetrainHardware = new DrivetrainSwerve();
  private final DrivetrainSub drivetrainSub = new DrivetrainSub(drivetrainHardware);

  private final IntakeRequirments intakeIOHardware = new IntakeHardware();
  private final IntakeSub intakeSub = new IntakeSub(intakeIOHardware);

  private final ArmRequirments armHardware = new ArmHardware();
  private final ArmSub armSub = new ArmSub(armHardware);

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
  }

  // this part should be the state machin
  public void configerButtonBindings() {
    drivetrainSub.setDefaultCommand(drivetrainSub.drive(cDriver));

    cRobotButtons.zeroSensors().whileTrue(intakeSub.intakePice());
    cRobotButtons.zeroSensors().onFalse(intakeSub.stopIntake());

    cOporator.intakePice().onTrue(armSub.IntakePosition());
    cOporator.OuttakePice().onFalse(armSub.OutakePositon());
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
  public void autonomousInit() {}

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
