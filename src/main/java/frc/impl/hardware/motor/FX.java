package frc.impl.hardware.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import frc.hardware.ProfiledMotor;

public class FX implements ProfiledMotor {
	final CoreTalonFX inner;
	final StatusSignal<Double> position;
	final StatusSignal<Double> velocity;

	public FX(TalonFXConfiguration conf, int id) {
		inner = new CoreTalonFX(id);
		inner.getConfigurator().apply(conf);

		position = inner.getPosition();
		velocity = inner.getVelocity();
	}

	@Override
	public double position() {
		return position.refresh().getValueAsDouble();
	}

	@Override
	public double velocity() {
		return velocity.refresh().getValueAsDouble();
	}

	@Override
	public void setPosition(double position) {
		inner.setControl(new MotionMagicTorqueCurrentFOC(position));

	}

	@Override
	public void setVelocity(double velocity) {
		inner.setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity));
	}
}
