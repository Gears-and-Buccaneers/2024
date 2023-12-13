package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    /** Contains all of the input data received from hardware. */
    @AutoLog
    public static class IntakeIOInputs {
        public double motorAppliedVolts = 0.0;
        public double[] motorCurrentAmps = new double[] {};
        public double[] motorTempCelcius = new double[] {};
        public boolean isDeployed = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run the gripper open loop at the specified voltage. [-12,12] */
    public default void setVoltage(double volts) {
    }

    public default void extend() {
    }

    public default void retract() {
    }

    /** Enable or disable brake mode on the gripper. */
    public default void setBrakeMode(boolean enable) {
    }
}
