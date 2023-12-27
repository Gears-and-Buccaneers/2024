package frc.robot.Subsytems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
// import frc.lib.hardware.motorController.*;

public class IntakeIOHardware implements IntakeRequirments {
  private final TalonSRX motor;

  private double setVolts = .1;

  public IntakeIOHardware() {
    motor = new TalonSRX(10);

    initPrefrences();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorAppliedVolts = motor.getSupplyCurrent() * motor.getBusVoltage();
    inputs.motorCurrentAmps = new double[] { motor.getSupplyCurrent() };
    // inputs.motorTempCelcius = new double[] {motor.getMotorTemperature()};
    // inputs.MotorVoltsOutput = motor.getVolts();
  }

  public void setIntakeVoltage() {
    motor.set(ControlMode.PercentOutput, setVolts);
  }

  public void setOutakeVoltage() {
    motor.set(ControlMode.PercentOutput, -setVolts);
  }

  @Override
  public void off() {
    motor.set(ControlMode.Disabled, 0);
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    System.out.println("cant close");
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }
}
