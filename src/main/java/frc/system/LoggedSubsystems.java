package frc.system;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface LoggedSubsystems extends Subsystem, AutoCloseable {
    // This is adding room to expand latter and makes sure all of are subsystems are
    // loggable
    void log();

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful
     * for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}.
     * Teams should try
     * to be consistent within their own codebases about which responsibilities will
     * be handled by
     * Commands, and which will be handled here.
     */
    default void periodic() {
        this.log();
    }
}
