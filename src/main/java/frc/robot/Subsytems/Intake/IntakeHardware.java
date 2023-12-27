package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
// import frc.lib.hardware.motorController.*;

public class IntakeHardware implements IntakeRequirments {
  private final TalonSRX motor;

  private double setVolts = .1;

  public IntakeHardware() {
    motor = new TalonSRX(10);

    initPrefrences();
  }

  public void setIntakeVoltage() {
    motor.set(ControlMode.PercentOutput, setVolts);
  }

  public void setOutakeVoltage() {
    motor.set(ControlMode.PercentOutput, -setVolts);
  }

  @Override
  public void off() {
    motor.set(ControlMode.Disabled, 0);
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    DriverStation.reportWarning("Canot close this type of motor", true); //TODO fix this
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("SupplyCurrent", motor.getSupplyCurrent());
  }

  @Override
  public void fromLog(LogTable table) {

  }
}
