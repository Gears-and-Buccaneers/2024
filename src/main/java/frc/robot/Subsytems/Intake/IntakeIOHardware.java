package frc.robot.Subsytems.Intake;

import frc.lib.hardware.motorController.*;

public class IntakeIOHardware implements IntakeRequirments {
  private final EncodedMotor motor;

  public IntakeIOHardware() {
    motor = new tallonFX(10);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
    inputs.motorTempCelcius = new double[] {motor.getMotorTemperature()};
    inputs.MotorVoltsOutput = motor.getVolts();
  }

  public void setVoltage(double speed) {
    motor.setVolts(speed / 12);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
