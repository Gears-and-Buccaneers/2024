package frc.system;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Vision {
	void with(Consumer<Measurement> measurement);

	public class Measurement {
		Pose2d pose;
		double timestamp;
		Matrix<N3, N1> stdDev;

		public Pose2d pose() {
			return pose;
		}

		public double timestamp() {
			return timestamp;
		}

		public Matrix<N3, N1> stdDev() {
			return stdDev;
		}
	}
}
