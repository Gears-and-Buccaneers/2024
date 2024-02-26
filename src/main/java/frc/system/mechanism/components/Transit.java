package frc.system.mechanism.components;

import frc.hardware.Motor;
import frc.system.mechanism.MotorComponent;

public class Transit extends MotorComponent {
	Transit(Motor motor) {
		super(motor, "Mechanism/Transit/Output", 0.4);
	}
}
