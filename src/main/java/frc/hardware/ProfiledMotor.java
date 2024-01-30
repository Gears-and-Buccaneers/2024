package frc.hardware;

public interface ProfiledMotor {
	public double position();

	public double velocity();

	public void setPosition(double position);

	public void setVelocity(double velocity);
}
