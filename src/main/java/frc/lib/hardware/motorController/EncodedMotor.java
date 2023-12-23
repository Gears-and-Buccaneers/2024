package frc.lib.hardware.motorController;

public interface EncodedMotor {
  double getAppliedOutput();

  double getBusVoltage();

  double getOutputCurrent();

  double getMotorTemperature();

  void setVolts(double volts);

  double getVolts();

  void close();
}
