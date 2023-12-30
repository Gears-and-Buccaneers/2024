package frc.lib.hardware.sensor.encoders;

import org.littletonrobotics.junction.LogTable;

public class REVBoreEncoder implements Encoder {

  @Override
  public double getPositoin() {
    return 20;
  }

  @Override
  public double getVelocity() {
    return 20;
  }

  @Override
  public void setGearRatio(double gearRatio) {
    // TODO Auto-generated method stub
  }

  // ----------------------------------------------------------
  @Override
  public void toLog(LogTable table) {
    table.put("Open", 1);
  }

  @Override
  public boolean connected() {
    return true; // TODO probaly Should FIx
  }

  // Unit Testing
  @Override
  public void close() throws Exception {}

  @Override
  public Encoder config() {

    return this;
  }
}
