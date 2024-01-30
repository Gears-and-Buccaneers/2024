package frc.lib.hardware.sensor.encoders;

import org.littletonrobotics.junction.LogTable;

public class REVBoreEncoder implements Encoder {

  @Override
  public double getPosition() {
    return 20;
  }

  @Override
  public double getVelocity() {
    return 20;
  }

  // ----------------------------------------------------------
  @Override
  public void toLog(LogTable table) {
    table.put("Open", 1);
  }

  @Override
  public boolean connected() {
    return true; // TODO probably should fix
  }

  // Unit Testing
  @Override
  public void close() throws Exception {}

  @Override
  public void zero() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zero'");
  }

  @Override
  public Encoder setInverted(boolean inverted) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
  }

  @Override
  public Encoder setOffset(double offset) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setOffset'");
  }

  @Override
  public Encoder setGearRatio(double gearRatio) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setGearRatio'");
  }
}
