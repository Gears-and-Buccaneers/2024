package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class Xbox extends LogitechController implements Driver, Operator {

  public Xbox(int port) {
    super(port);
  }

  @Override
  public Trigger intakePiece() {
    return RB;
  }

  @Override
  public Trigger outtakeTopPiece() {
    return LB;
  }

  @Override
  public Trigger outtakeMidPiece() {
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
  public double getDrivetrainTranslationX() {
    return -LS_Y.get();
  }

  @Override
  public double getDrivetrainTranslationY() {
    return LS_X.get();
  }

  @Override
  public double getDrivetrainRotation() {
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
