package frc.robot.Subsytems.Intake;

import frc.robot.Subsytems.SubsytemRequirments;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRequirments extends SubsytemRequirments {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double motorAppliedVolts = 0.0;
    public double[] motorCurrentAmps = new double[] {};
    public double[] motorTempCelcius = new double[] {};
    public double MotorVoltsOutput = 0.0; // for unit test
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  // Controling the hardware
  public void setOutakeVoltage();

  public void setIntakeVoltage();

  public void off();

  /** Enable or disable brake mode on the intake. */
  public default void setBrakeMode(boolean enable) {}
}
