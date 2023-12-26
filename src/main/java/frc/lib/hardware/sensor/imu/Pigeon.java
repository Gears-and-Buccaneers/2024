package frc.lib.hardware.sensor.imu;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon implements IMU {

  //Hardware Getting and setting methods-------------------------------

  @Override
  public Rotation2d getPitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPitch'");
  }

  @Override
  public Rotation2d getYaw() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getYaw'");
  }

  @Override
  public Rotation2d getRoll() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRoll'");
  }

  @Override
  public void zeroIMU() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zeroIMU'");
  }

  //----------------------------------------------------------

  @Override
  public boolean connected() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'connected'");
  }

  @Override
  public void toLog(LogTable table) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'toLog'");
  }

  @Override
  public void fromLog(LogTable table) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void run() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'run'");
  }

  @Override
  public double getPeriod() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPeriod'");
  }
}
