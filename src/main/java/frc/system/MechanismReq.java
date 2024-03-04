package frc.system;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MechanismReq extends Subsystem, AutoCloseable {
    public void disable();

    public void log();

    default void periodic() {
        this.log();
    }
}