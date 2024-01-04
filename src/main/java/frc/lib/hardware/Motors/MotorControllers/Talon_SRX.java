package frc.lib.hardware.Motors.MotorControllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.littletonrobotics.junction.LogTable;

public class Talon_SRX implements MotorController {
  private int canID;

  private TalonSRX motor;

  public MotorController build(int canID) {
    this.canID = canID;

    motor = new TalonSRX(canID);

    return this;
  }

  @Override
  public void runVolts(double num) {
    motor.set(ControlMode.PercentOutput, num);
  }

  @Override
  public void brakeMode(boolean enable) {
    if (enable)
      motor.setNeutralMode(NeutralMode.Brake);
    else
      motor.setNeutralMode(NeutralMode.Coast);
  }


  public void disable() {
    motor.set(ControlMode.Disabled, 0);
  }

  public void currentLimit() {
  }

  // ----------------------------------------------------------
  @Override
  public int getCanID() {
    return canID;
  }

  @Override
  public void toLog(LogTable table, String path) {
    table.put(path + "/MotorController/Open", 1);
  }

  @Override
  public boolean connected() {
    return true; // TODO probably should fix
  }

  // Unit Testing
  @Override
  public void close() throws Exception {
  }

  @Override
  public double getAppliedVolts() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAppliedVolts'");
  }

  @Override
  public double getCurentAmps() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurentAmps'");
  }

  @Override
  public double getBusVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBusVoltage'");
  }

  @Override
  public MotorController setInverted(boolean enable) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
  }

  @Override
  public MotorController setCurrentLimit(double CurrentLimit) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
  }

  @Override
  public MotorController setSimulated(double simulated) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setSimulated'");
  }

}
