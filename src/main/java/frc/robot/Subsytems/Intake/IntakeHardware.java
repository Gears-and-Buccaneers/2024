package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
// import frc.lib.hardware.motorController.*;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class IntakeHardware implements IntakeRequirments {
  private final CANSparkMax motor;
  private final ProximitySwitch switch1;

  private double setVolts = .1;

  private String logName;

  public IntakeHardware() {
    motor = new CANSparkMax(15, MotorType.kBrushed);
    switch1 = new Huchoo(3);

    initPrefrences();
  }

  public void setIntakeVoltage() {
    System.out.println("stufffff");
    motor.set(setVolts);
    // motor.set(ControlMode.PercentOutput, setVolts);
  }

  public void setOutakeVoltage() {
    motor.set(-setVolts);
    // motor.set(ControlMode.PercentOutput, -setVolts);
  }

  public void off() {
    motor.stopMotor();
    // motor.set(ControlMode.Disabled, 0);
  }

  public boolean isOpen() {
    return switch1.isOpen();
  }

  public boolean isClosed() {
    return switch1.isClosed();
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    switch1.close();
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("SupplyCurrent", 5);
    Logger.processInputs(logName + "/switch1", switch1);
  }

  @Override
  public void fromLog(LogTable table) {

  }

  public void setLogName(String logName) {
    this.logName = logName;
  }
}
