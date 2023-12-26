package frc.lib.hardware.sensor;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import frc.lib.hardware.sim.SimProfile;

public interface SensorRequirments extends AutoCloseable, LoggableInputs, SimProfile {

    boolean connected();

}
