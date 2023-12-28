package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.wpilibj.Preferences;

import frc.lib.hardware.motorController.*;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class IntakeHardware implements IntakeRequirments {
  private final Motor motor;
  private final ProximitySwitch switch1;

  private double setVolts = .1;

  private String SimpleName;

  public IntakeHardware() {
    motor = new VP775(10);
    switch1 = new Huchoo(3);

    initPrefrences();
  }

  public void setIntakeSpeed() {

  }

  public void setOutakeSpeed() {

  }

  public boolean isOpen() {
    return switch1.isOpen();
  }

  public boolean isClosed() {
    return switch1.isClosed();
  }

  // required things-------------------------------------------------
  public void periodic() {
    loadPreferences();
  }

  public void disable() {
    motor.disable();
  }

  // prefrances ------------------
  private void initPrefrences() {
    Preferences.initDouble(SimpleName + "/maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble(SimpleName + "/maxVolts", setVolts);
  }

  // Loging ------------------------------------
  @Override
  public void toLog(LogTable table) {
  }

  public void setSimpleName(String SimpleName) {
    this.SimpleName = SimpleName;
  }

  // Auto CLosing -------------------------------
  @Override
  public void close() throws Exception {
    switch1.close();
  }
}
