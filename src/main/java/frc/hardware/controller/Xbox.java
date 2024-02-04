package frc.hardware.controller;

import edu.wpi.first.wpilibj.XboxController;
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
	public final Button lB = new Button(XboxController.Button.kLeftBumper.value);
	/** Right bumper. */
	public final Button rB = new Button(XboxController.Button.kRightBumper.value);
	/** Left stick down. */
	public final Button lS = new Button(XboxController.Button.kLeftStick.value);
	/** Right stick down. */
	public final Button rS = new Button(XboxController.Button.kRightStick.value);
	/** A. */
	public final Button a = new Button(XboxController.Button.kA.value);
	/** B. */
	public final Button b = new Button(XboxController.Button.kB.value);
	/** X. */
	public final Button x = new Button(XboxController.Button.kX.value);
	/** Y. */
	public final Button y = new Button(XboxController.Button.kY.value);
	/** Back. */
	public final Button back = new Button(XboxController.Button.kBack.value);
	/** Start. */
	public final Button start = new Button(XboxController.Button.kStart.value);

	/** Construct a new Xbox controller. */
	public Xbox(int port) {
		super(port);
	}
}
