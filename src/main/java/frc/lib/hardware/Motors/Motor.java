package frc.lib.hardware.Motors;

import frc.lib.hardware.HardwareRequirments;
import frc.lib.hardware.Motors.MotorControlers.MotorController;
import frc.lib.hardware.Motors.MotorControlers.Talon_SRX;
import frc.lib.hardware.Motors.PID.EncoderConfigs;
import frc.lib.hardware.Motors.PID.PIDConfigs;
import frc.lib.hardware.sensor.encoders.Encoder;
import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class Motor implements HardwareRequirments {
  public enum Type {
    CIM,
    Falcon500,
    VP775
  }

  public enum ControllerType {
    TallonSRX(new Talon_SRX());

    MotorController mController;

    private ControllerType(MotorController mController) {
      this.mController = mController;
    }

    public MotorController config(int canID) {
      return mController.config(canID);
    }
  }

  private final MotorController mController;
  private final Type motorType;
  private Encoder encoder;
  private boolean hasEncoder = false;
  private String name;

  public Motor(ControllerType motorControllerType, int canID, Type motor) {
    this.mController = motorControllerType.config(canID);
    this.motorType = motor;
    this.name = "Motor" + canID;    
  }

  // ----------------- COnfigs ---------------------
  public Motor addEncoder(Encoder encoder) {
    this.encoder = encoder;
    hasEncoder = true;
    return this;
  }

  public Motor inverted(boolean enable) {
    mController.setInverted(enable);

    return this;
  }

  public Motor brakeMode(boolean enable) {
    mController.brakeMode(enable);

    return this;
  }

  public Motor pidConfigs(PIDConfigs PIDConfigs) {

    return this;
  }

  public Motor EncoderConfigs(EncoderConfigs encoderConfigs) {

    return this;
  }

  public Motor setName(String name) {
    this.name = name;
    return this;
  }

  // set ---------------------------------
  public void setPercentOut(double percentOut) {
    mController.runPercentOut(percentOut);
  }

  public void setPositoin(double Positoin) {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
  }

  public void disable() {
    mController.disable();
  }

  // get ---------------------------------
  public int getCanID() {
    return mController.getCanID();
  }

  @Override
  public void close() throws Exception {
    mController.close();
    encoder.close();
  }

  public double getPositoin() {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    return encoder.getPositoin();
  }

  // Required ---------------------------

  @Override
  public void toLog(LogTable table) {
    mController.toLog(table, name);
    table.put(name + "/Motor Type", motorType.toString());
  }

  @Override
  public boolean connected() {
    if (!mController.connected()) return false;
    if (hasEncoder) return encoder.connected();
    return true;
  }
}
