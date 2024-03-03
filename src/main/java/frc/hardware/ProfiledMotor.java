package frc.hardware;

public interface ProfiledMotor extends Motor {
    /** Gets the measured position of the motor. */
    public double position();

    /** Gets the measured velocity of the motor. */
    public double velocity();

    /** Sets the motor setpoint to the specified position. */
    public void setPosition(double position);

    /** Sets the motor setpoint to the specified velocity. */
    public void setVelocity(double velocity);

    /** Sets the motor setpoint relative to the current position. */
    default public void setRelativePosition(double dPosition) {
        setPosition(position() + dPosition);
    }

}
