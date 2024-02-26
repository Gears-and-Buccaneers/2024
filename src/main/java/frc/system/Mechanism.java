package frc.system;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Mechanism extends Subsystem {
	Command intake();

	Command aim();

	Command shoot();
}
