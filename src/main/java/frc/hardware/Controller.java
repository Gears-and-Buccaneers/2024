package frc.hardware;

import edu.wpi.first.wpilibj.GenericHID;

public abstract class Controller {
	/** The internal controller. */
	final GenericHID hid;

	public Controller(int port) {
		hid = new GenericHID(port);
	}

	public class Button {
		final int id;

		public Button(int id) {
			this.id = id;
		}

		/** Get whether the button is pressed. */
		public boolean get() {
			return hid.getRawButton(id);
		}
	}

	public class Axis {
		final int id;

		public Axis(int id) {
			this.id = id;
		}

		/** Get the value that this input is at, from -1.0 to +1.0. */
		public double get() {
			return hid.getRawAxis(id);
		}
	}
}
