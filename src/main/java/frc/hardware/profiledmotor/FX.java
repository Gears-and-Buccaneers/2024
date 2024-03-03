package frc.hardware.profiledmotor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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

    public void follow(FX other, boolean inverted) {
        inner.setControl(new Follower(other.inner.getDeviceID(), inverted));
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

    @Override
    public int getDeviceID() {
        return inner.getDeviceID();
    }

    @Override
    public String getControlMode() {
        return inner.getControlMode().toString();
    }

    @Override
    public double getDeviceTemp() {
        return inner.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        return inner.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return inner.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getMotorVoltage() {
        return inner.getMotorVoltage().getValueAsDouble();
    }
}
