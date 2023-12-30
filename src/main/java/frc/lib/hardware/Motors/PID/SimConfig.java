package frc.lib.hardware.Motors.PID;

public class SimConfig {
  private final boolean simVelocity;
  private final double gearRatio;
  private final double p;

  public SimConfig(boolean simVelocity, double gearRatio, double p) {
    this.simVelocity = simVelocity;
    this.gearRatio = gearRatio;
    this.p = p;
  }

  public boolean simVelocity() {
    return simVelocity;
  }

  public double gearRatio() {
    return gearRatio;
  }

  public double getP() {
    return p;
  }
}
