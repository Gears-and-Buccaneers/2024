package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class Xbox extends LogitechController implements Driver, Oporator {

  public Xbox(int port) {
    super(port);
  }

  @Override
  public Trigger intakePice() {
    return RB;
  }

  @Override
  public Trigger OuttakeTopPice() {
    return LB;
  }

  @Override
  public Trigger OuttakeMidPice() {
    return new Trigger(
        () -> {
          return LT_S.get() >= .2;
        });
  }

  @Override
  public Trigger setPose() {
    return A;
  }

  @Override
  public double getDrivtrainTranslationX() {
    return -LS_Y.get();
  }

  @Override
  public double getDrivtrainTranslationY() {
    return LS_X.get();
  }

  @Override
  public double getDrivtrainRotation() {
    return -RS_X.get();
  }

  @Override
  public Trigger zeroGyro() {
    return BACK;
  }

  public Command rumble() {
    return super.Rumble;
  }
}
