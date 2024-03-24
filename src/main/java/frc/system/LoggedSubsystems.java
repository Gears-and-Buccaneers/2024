package frc.system;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LoggedSubsystems extends Subsystem, AutoCloseable{
    //This is adding room to expand latter and makes sure all of are subsystems are loggable
    void log();
}
