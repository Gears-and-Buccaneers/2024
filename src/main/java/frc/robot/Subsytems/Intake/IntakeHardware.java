package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;

import frc.lib.hardware.motorController.*;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class IntakeHardware implements IntakeRequirments {
  private final Motor motor;
  private final ProximitySwitch switch1;

  private double setPercentOut = .1;

  private String SimpleName;

  public IntakeHardware() {
    motor = new VP775(10);
    switch1 = new Huchoo(3);

    initPrefrences();
  }

  public void setIntakeSpeed() {
    motor.runPercentOut(setPercentOut);
  }

  public void setOutakeSpeed() {
    motor.runPercentOut(-setPercentOut);
  }

  public boolean isOpen() {
    return switch1.isOpen();
  }

  public boolean isClosed() {
    return switch1.isClosed();
  }

  public void setBrakeMode(boolean enable) {
    motor.brakeMode(enable);
  }

  // required things-------------------------------------------------
  public void periodic() {
    loadPreferences();
  }

  public void disable() {
    motor.disable();
  }

  // Loging ------------------------------------
  @Override
  public void toLog(LogTable table) {
    table.put("setPercentOut", setPercentOut);
    Logger.processInputs(SimpleName + "/Motor" + motor.getCanID(), motor);
    Logger.processInputs(SimpleName + "/ProximitySwitch" + switch1.getDIOChannel(), switch1);
  }

  public void setSimpleName(String SimpleName) {
    this.SimpleName = SimpleName;
  }

  // Auto CLosing -------------------------------
  @Override
  public void close() throws Exception {
    switch1.close();
  }

  // prefrances ------------------
  private void initPrefrences() {
    Preferences.initDouble(SimpleName + "/maxVolts", setPercentOut);
  }

  public void loadPreferences() {
    setPercentOut = Preferences.getDouble(SimpleName + "/maxVolts", setPercentOut);
  }
}
