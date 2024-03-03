package frc.hardware;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class Controller {
	/** The internal controller. */
	private final CommandGenericHID hid;

	public Controller(int port) {
		hid = new CommandGenericHID(port);
	}

	protected Trigger button(int id) {
		return hid.button(id);
	}

	public class Axis {
		final int id;

		public Axis(int id) {
			this.id = id;
		}

		/**
		 * Returns a new BooleanEvent which can be used for command scheduling, taking
		 * an activation threshold.
		 */
		public Trigger event(double threshold) {
			return hid.axisGreaterThan(id, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
		}

		/** Get the value that this input is at, from -1.0 to +1.0. */
		public double get() {
			return hid.getRawAxis(id);
		}
	}
}
