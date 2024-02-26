package frc.hardware.profiledmotor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import frc.hardware.ProfiledMotor;

public class FX implements ProfiledMotor {
	final CoreTalonFX inner;
	final StatusSignal<Double> position;
	final StatusSignal<Double> velocity;

	public FX(int id) {
		inner = new CoreTalonFX(id);

		position = inner.getPosition();
		velocity = inner.getVelocity();
	}

	public FX with(TalonFXConfiguration conf) {
		inner.getConfigurator().apply(conf);
		return this;
	}

	public FX with(MotorOutputConfigs conf) {
		inner.getConfigurator().apply(conf);
		return this;
	}

	public FX with(FeedbackConfigs conf) {
		inner.getConfigurator().apply(conf);
		return this;
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
		inner.setControl(new PositionDutyCycle(position));
		// TODO: switch to Phoenix Pro control mode:
		// inner.setControl(new MotionMagicTorqueCurrentFOC(position));
	}

	@Override
	public void setVelocity(double velocity) {
		System.out.println("Velocity set to " + velocity);
		inner.setControl(new VelocityDutyCycle(velocity));
		// TODO: switch to Phoenix Pro control mode
		// inner.setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity));
	}

	@Override
	public void setPercent(double percent) {
		inner.setControl(new DutyCycleOut(percent));
	}

	@Override
	public void setVoltage(double voltage) {
		inner.setControl(new VoltageOut(voltage));
	}
}
