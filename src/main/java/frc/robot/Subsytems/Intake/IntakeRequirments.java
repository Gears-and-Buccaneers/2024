package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRequirments extends AutoCloseable {
    /** Contains all of the input data received from hardware. */
    @AutoLog
    public static class IntakeIOInputs {
        public double motorAppliedVolts = 0.0;
        public double[] motorCurrentAmps = new double[] {};
        public double[] motorTempCelcius = new double[] {};
        public boolean isDeployed = false; // for unit test
        public double MotorVoltsOutput = 0.0; // for unit test
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    // Controling the hardware
    /** Run the gripper open loop at the specified voltage. [-12,12] */
    public void setVoltage(double volts);

    /** Exstends the intake */
    public void extend();

    /** retracts the intake */
    public void retract();

    /** Enable or disable brake mode on the intake. */
    public default void setBrakeMode(boolean enable) {
    }
}
