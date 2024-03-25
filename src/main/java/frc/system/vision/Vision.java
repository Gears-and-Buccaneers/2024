package frc.system.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Vision {
	/** Registers a consumer which is called when measurements are available. */
	void register(Consumer<Measurement> measurement);

	/** A single vision measurement of an object. */
	public class Measurement {
		public Pose3d pose;
		public double timestamp;
		public Matrix<N3, N1> stdDev;

		/** Gets the pose of the object. */
		public Pose3d pose() {
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
