package frc.lib.hardware.sensor;

import frc.lib.hardware.sim.SimProfile;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SensorRequirments extends AutoCloseable, LoggableInputs, SimProfile {

  boolean connected();
}
