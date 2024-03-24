package frc.system;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LoggedSubsystems extends Subsystem, AutoCloseable{
    void log();
}
