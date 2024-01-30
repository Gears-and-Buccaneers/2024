package frc.hardware.encoder;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import frc.hardware.Encoder;

public class CANcoder implements Encoder {
	final CoreCANcoder inner;

	public CANcoder(int id) {
		inner = new CoreCANcoder(id);
	}

	@Override
	public double velocity() {
		return inner.getVelocity().refresh().getValueAsDouble();
	}

	@Override
	public double position() {
		return inner.getPosition().refresh().getValueAsDouble();
	}
}
