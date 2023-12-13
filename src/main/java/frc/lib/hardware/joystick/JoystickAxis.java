package frc.lib.hardware.joystick;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class JoystickAxis implements Supplier<Double> {
	private final double deadband;
	public SlewRateLimiter limiter;

	private final Supplier<Double> axis;

	public JoystickAxis(Joystick joystick, int axis, double deadband, double rateLimit) {
		this.axis = () -> joystick.getRawAxis(axis);
		this.deadband = deadband;
		limiter = new SlewRateLimiter(rateLimit);
	}

	public JoystickAxis(XboxController joystick, XboxController.Axis axis, double deadband, double rateLimit) {
		this.axis = () -> joystick.getRawAxis(axis.value);
		this.deadband = deadband;
		limiter = new SlewRateLimiter(rateLimit);
	}

	public Double get() {
		double val = axis.get();
		if (Math.abs(val) < deadband) {
			return 0.0;
		}

		return limiter.calculate(val);
	}
}