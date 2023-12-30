package frc.lib.hardware.Motors.PID;

public class SimConfig {
  private final boolean simVelocity;
  private final double gearRatio;

  public SimConfig(boolean simVelocity, double gearRatio) {
    this.simVelocity = simVelocity;
    this.gearRatio = gearRatio;
  }

  public boolean simVelocity() {
    return simVelocity;
  }

  public double gearRatio() {
    return gearRatio;
  }
}
