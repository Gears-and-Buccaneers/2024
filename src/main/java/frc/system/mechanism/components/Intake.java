package frc.system.mechanism.components;

import frc.hardware.Motor;
import frc.system.mechanism.MotorComponent;

public class Intake extends MotorComponent {
	public Intake(Motor motor, double defaultOutput) {
		super(motor, "Mechanism/Intake/Output", defaultOutput);
	}
}
