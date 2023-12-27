package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.hardware.sensor.proximitySwitch.Huchoo;
import frc.lib.hardware.sensor.proximitySwitch.ProximitySwitch;
import frc.robot.Subsytems.Intake.*;
import frc.robot.joystics.*;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/gripper/GripperIO.java
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md
// started to implment advantage kit
// Just testing something
public class Robot extends LoggedRobot {
  // Controlers
  private Oporator oporator;
  private RobotButtons robotButtons;

  //subsytems
  private IntakeSub intakeSub;
  private IntakeRequirments intakeIOHardware;
  private ProximitySwitch intakeProximitySwitch;


  @Override
  public void robotInit() {
    //Logging
    Logger.recordMetadata("ProjectName", "MyProject");
    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.disableDeterministicTimestamps();
    Logger.start();

    //Subsytems
    intakeIOHardware = new IntakeIOHardware();
    intakeProximitySwitch = new Huchoo(3);
    intakeSub = new IntakeSub(intakeIOHardware, intakeProximitySwitch);

    // Controlers
    robotButtons = new RealRobotButtons();
    oporator = new SamKeyboard(0);

    //Button Bindings
    configerButtonBindings();
  }

  public void configerButtonBindings() {
    robotButtons.zeroSensors().whileTrue(intakeSub.intakePice());
    robotButtons.zeroSensors().onFalse(intakeSub.stopIntake());

    oporator.OuttakePice().whileTrue(intakeSub.ejectPice());
    oporator.OuttakePice().onFalse(intakeSub.stopIntake());
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
}
