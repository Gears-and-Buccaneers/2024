package frc.lib.hardware.sensor.proximitySwitch;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch implements ProximitySwitch {
  private DigitalInput limitSwitch;
  private int DIOChannel;

  /**
   * @param DIOChannel the DIO channel for the digital input 0-9 are on-board
   */
  public LimitSwitch(int DIOChannel) {
    this.DIOChannel = DIOChannel;
    limitSwitch = new DigitalInput(DIOChannel);
    System.out.println("[init] Limit switch on port " + DIOChannel);
  }

  @Override
  public boolean get() {
    return limitSwitch.get();
  }

  // ----------------------------------------------------------
  @Override
  public boolean connected() {
    return true; // TODO probaly Should FIx
  }

  // Unit Testing
  @Override
  public void close() throws Exception {
    limitSwitch.close();
  }

  // Simulating
  @Override
  public void run() {
    // TODO work on this
    limitSwitch.setSimDevice(null);
  }

  private double _lastTime;
  private boolean _running = false;

  @Override
  public double getPeriod() {
    if (!_running) {
      _lastTime = Utils.getCurrentTimeSeconds();
      _running = true;
    }

    double now = Utils.getCurrentTimeSeconds();
    final double period = now - _lastTime;
    _lastTime = now;

    return period;
  }
}
