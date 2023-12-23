package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class DefaultLogitech extends LogitechController implements Oporator {
  public DefaultLogitech(int port) {
    super(port);
  }

  @Override
  public Trigger intakePice() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'intakePice'");
  }

  @Override
  public Trigger OuttakePice() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'OuttakePice'");
  }

  @Override
  public double getDrivtrainTranslationX() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivtrainTranslationX'");
  }

  @Override
  public double getDrivtrainTranslationY() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivtrainTranslationY'");
  }

  @Override
  public double getDrivtrainRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivtrainRotation'");
  }

}
