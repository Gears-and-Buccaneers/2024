package frc.hardware.profiledmotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.hardware.Encoder;
import frc.hardware.Motor;
import frc.hardware.ProfiledMotor;

/**
 * Fuses a motor without encoder capabilities with an external encoder and PID
 * controllers.
 */
public class Fused implements ProfiledMotor {
    final Encoder encoder;
    final Motor motor;

    final ProfiledPIDController posControl;
    final ProfiledPIDController velControl;

    ControlMode mode;

    enum ControlMode {
        Position,
        Velocity;
    }

    public static class kPID {
        public final double k;
        public final double i;
        public final double d;

        public kPID(double k, double i, double d) {
            this.k = k;
            this.i = i;
            this.d = d;
        }
    }

    public Fused(Motor motor, Encoder encoder, kPID pos, kPID vel, double velMax, double accMax, double jerkMax) {
        this.motor = motor;
        this.encoder = encoder;

        this.posControl = new ProfiledPIDController(pos.k, pos.i, pos.d,
                new TrapezoidProfile.Constraints(velMax, accMax));
        this.velControl = new ProfiledPIDController(vel.k, vel.i, vel.d,
                new TrapezoidProfile.Constraints(accMax, jerkMax));

        CommandScheduler.getInstance().schedule(new Command() {
            @Override
            public void execute() {
                if (mode == ControlMode.Position) {
                    motor.setVoltage(posControl.calculate(encoder.position()));
                } else if (mode == ControlMode.Velocity) {
                    motor.setVoltage(velControl.calculate(encoder.velocity()));
                }
            }
        });
    }

    @Override
    public double position() {
        return encoder.position();
    }

    @Override
    public double velocity() {
        return encoder.velocity();
    }

    @Override
    public void setPosition(double position) {
        posControl.setGoal(position);
        mode = ControlMode.Position;
    }

    @Override
    public void setVelocity(double velocity) {
        velControl.setGoal(velocity);
        mode = ControlMode.Velocity;
    }

    @Override
    public void setPercent(double percent) {
        motor.setPercent(percent);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public int getDeviceID() {
        return 0;
    }

    @Override
    public String getControlMode() {
        throw new UnsupportedOperationException("Unimplemented method 'getControlMode'");
    }

    @Override
    public double getDeviceTemp() {
        throw new UnsupportedOperationException("Unimplemented method 'getDeviceTemp'");
    }

    @Override
    public double getSupplyCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getSupplyCurrent'");
    }

    @Override
    public double getStatorCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getStatorCurrent'");
    }

    @Override
    public double getMotorVoltage() {
        throw new UnsupportedOperationException("Unimplemented method 'getMotorVoltage'");
    }
}
