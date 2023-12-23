package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj.Preferences;
import frc.lib.hardware.motorController.*;

public class IntakeIOHardware implements IntakeRequirments {
  private final Motor motor;

  private double setVolts = 6;

  public IntakeIOHardware() {
    motor = new TalonFXGB(10);

    initPrefrences();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    // inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
    // inputs.motorTempCelcius = new double[] {motor.getMotorTemperature()};
    // inputs.MotorVoltsOutput = motor.getVolts();
  }

  public void setIntakeVoltage() {
    motor.setVoltageOut(setVolts);
  }

  public void setOutakeVoltage() {
    motor.setVoltageOut(-setVolts);
  }

  @Override
  public void off() {
    motor.setVoltageOut(0);
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    motor.close();
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }
}
