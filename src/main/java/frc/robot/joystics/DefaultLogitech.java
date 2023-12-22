package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class DefaultLogitech extends LogitechController implements Oporator {
  public DefaultLogitech(int port) {
    super(port);
  }

  @Override
  public Trigger intakePice() {
    return LB;
  }

  @Override
  public Trigger OuttakePice() {
    return X;
  }
}
