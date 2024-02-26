package frc.system.mechanism.components;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.hardware.Motor;
import frc.system.mechanism.MotorComponent;

public class Shooter extends MotorComponent {
	Shooter(Motor motor) {
		super(motor, "Mechanism/Shooter/Output", 1);
	}

	public Command waitPrimed() {
		return new WaitCommand(1);
	}
}
