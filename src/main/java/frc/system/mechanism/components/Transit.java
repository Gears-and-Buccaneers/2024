package frc.system.mechanism.components;

import frc.hardware.Motor;
import frc.system.mechanism.MotorComponent;

public class Transit extends MotorComponent {
	public Transit(Motor motor, double defaultOutput) {
		super(motor, "Mechanism/Transit/Output", defaultOutput);
	}
}
