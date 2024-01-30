package frc.impl.hardware.encoder;

import frc.hardware.Encoder;

public class WPI implements Encoder {
	final edu.wpi.first.wpilibj.Encoder inner;

	public WPI(int a, int b) {
		inner = new edu.wpi.first.wpilibj.Encoder(a, b);
	}

	@Override
	public double velocity() {
		return inner.getRate();
	}

	@Override
	public double position() {
		return inner.getRaw();
	}
}
