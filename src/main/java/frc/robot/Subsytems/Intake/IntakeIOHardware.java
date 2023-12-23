package frc.robot.Subsytems.Intake;

import frc.lib.hardware.motorController.*;

public class IntakeIOHardware implements IntakeRequirments {
  private final Motor motor;

  public IntakeIOHardware() {
    motor = new TalonFXGB(10);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    // inputs.motorCurrentAmps = new double[] {motor.getOutputCurrent()};
    // inputs.motorTempCelcius = new double[] {motor.getMotorTemperature()};
    // inputs.MotorVoltsOutput = motor.getVolts();
  }

  public void setVoltage(double speed) {
    motor.setVoltageOut(speed);
  }

  @Override
  public void close() throws Exception {
    motor.close();
  }
}
