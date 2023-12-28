package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Subsytems.Arm.*;
import frc.robot.Subsytems.Intake.*;
import frc.robot.joystics.*;

public class Robot extends LoggedRobot {
  // Controlers
  private Oporator oporator;
  private RobotButtons robotButtons;

  // subsytems
  private IntakeSub intakeSub;
  private IntakeRequirments intakeIOHardware;

  // private ArmSub armSub;
  // private ArmRequirments armHardware;

  @Override
  public void robotInit() {
    // Logging
    Logger.recordMetadata("ProjectName", "MyProject");

    Logger.addDataReceiver(new NT4Publisher());
    Logger.disableDeterministicTimestamps();
    Logger.start();

    // Subsytems
    intakeIOHardware = new IntakeHardware();
    intakeSub = new IntakeSub(intakeIOHardware);

    // armHardware = new ArmHardware();
    // armSub = new ArmSub(armHardware);

    // Controlers
    robotButtons = new RealRobotButtons();
    oporator = new SamKeyboard(0);

    // Button Bindings
    configerButtonBindings();
  }

  // this part should be the state machin
  public void configerButtonBindings() {
    robotButtons.zeroSensors().whileTrue(intakeSub.intakePice());
    robotButtons.zeroSensors().onFalse(intakeSub.stopIntake());

    // oporator.intakePice().onTrue(armSub.IntakePosition());
    // oporator.OuttakePice().onFalse(armSub.OutakePositon());
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
