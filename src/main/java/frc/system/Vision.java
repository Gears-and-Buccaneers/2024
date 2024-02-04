package frc.system;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Vision {
	/** Registers a consumer which is called when measurements are available. */
	void with(Consumer<Measurement> measurement);

	/** A single vision measurement of an object. */
	public class Measurement {
		Pose2d pose;
		double timestamp;
		Matrix<N3, N1> stdDev;

		/** Gets the pose of the object. */
		public Pose2d pose() {
			return pose;
		}

		/** Gets the timestamp at which this measurement was made. */
		public double timestamp() {
			return timestamp;
		}

		/** The x, y, and theta standard deviations of the measurement result. */
		public Matrix<N3, N1> stdDev() {
			return stdDev;
		}
	}
}
