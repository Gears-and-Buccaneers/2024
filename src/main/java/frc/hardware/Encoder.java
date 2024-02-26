package frc.hardware;

public interface Encoder {
	/** Gets the measured velocity of the encoder. */
	double velocity();

	/** Gets the measured position of the encoder. */
	double position();
}
