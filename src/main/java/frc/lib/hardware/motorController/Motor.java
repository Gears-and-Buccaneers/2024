package frc.lib.hardware.motorController;

public interface Motor extends AutoCloseable {
  boolean simulated = false;

  void setVoltageOut(double voltageOut);

  void setPercentOut(double percentOut);
}
