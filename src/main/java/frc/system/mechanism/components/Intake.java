package frc.system.mechanism.components;

import frc.hardware.Motor;
import frc.system.mechanism.MotorComponent;

public class Intake extends MotorComponent {
	Intake(Motor motor) {
		super(motor, "Mechanism/Intake/Output", 0.6);
	}
}
