package frc.hardware.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hardware.Controller;

public class Xbox extends Controller {
	/** Left stick x-axis. */
	public final Axis lX = new Axis(XboxController.Axis.kLeftX.value);
	/** Left stick y-axis. */
	public final Axis lY = new Axis(XboxController.Axis.kLeftY.value);
	/** Right stick x-axis. */
	public final Axis rX = new Axis(XboxController.Axis.kRightX.value);
	/** Right stick y-axis. */
	public final Axis rY = new Axis(XboxController.Axis.kRightY.value);
	/** Left trigger. */
	public final Axis lT = new Axis(XboxController.Axis.kLeftTrigger.value);
	/** Right trigger. */
	public final Axis rT = new Axis(XboxController.Axis.kRightTrigger.value);
	/** Left bumper. */
	public final Trigger lB = button(XboxController.Button.kLeftBumper.value);
	/** Right bumper. */
	public final Trigger rB = button(XboxController.Button.kRightBumper.value);
	/** Left stick down. */
	public final Trigger lS = button(XboxController.Button.kLeftStick.value);
	/** Right stick down. */
	public final Trigger rS = button(XboxController.Button.kRightStick.value);
	/** A. */
	public final Trigger a = button(XboxController.Button.kA.value);
	/** B. */
	public final Trigger b = button(XboxController.Button.kB.value);
	/** X. */
	public final Trigger x = button(XboxController.Button.kX.value);
	/** Y. */
	public final Trigger y = button(XboxController.Button.kY.value);
	/** Back. */
	public final Trigger back = button(XboxController.Button.kBack.value);
	/** Start. */
	public final Trigger start = button(XboxController.Button.kStart.value);

	/** Construct a new Xbox controller. */
	public Xbox(int port) {
		super(port);
	}
}
