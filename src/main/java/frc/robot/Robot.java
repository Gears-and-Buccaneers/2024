package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Subsytems.Intake.*;
import frc.robot.joystics.*;

// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/gripper/GripperIO.java
// https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/INSTALLATION.md
// started to implment advantage kit
public class Robot extends LoggedRobot {
  private Oporator oporator = new SamLogitech(1);

  private IntakeSub intakeSub;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kCTRE); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    configerButtonBindings();

    intakeSub = new IntakeSub(new IntakeIOHardware());
  }

  public void configerButtonBindings() {
    oporator.intakePice().whileTrue(intakeSub.intakePice());
    oporator.intakePice().onFalse(intakeSub.retract().ignoringDisable(true));

    oporator.OuttakePice().whileTrue(intakeSub.ejectPice());
    oporator.OuttakePice().onFalse(intakeSub.retract());
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
