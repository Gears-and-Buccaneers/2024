package frc.command;

import edu.wpi.first.math.proto.Controller;
import frc.system.Drivetrain;

public class TeleOp {
	final Drivetrain drivetrain;
	final Controller controller;

	public TeleOp(Drivetrain drivetrain, Controller controller) {
		this.drivetrain = drivetrain;
		this.controller = controller;
	}
}
