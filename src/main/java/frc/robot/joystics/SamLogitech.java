package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class SamLogitech extends LogitechController implements Oporator {
  public SamLogitech(int port) {
    super(port);
  }

  @Override
  public Trigger intakePice() {
    return Y;
  }

  @Override
  public Trigger OuttakePice() {
    return A;
  }

  @Override
  public double getDrivtrainTranslationX() {
    return LS_X.get();
  }
}
