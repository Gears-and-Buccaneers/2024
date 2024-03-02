package frc.system;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.command.TeleOp;

public interface Mechanism extends Subsystem {
	Command intake();

	Command aim(TeleOp teleop);

	Command shoot();
}
