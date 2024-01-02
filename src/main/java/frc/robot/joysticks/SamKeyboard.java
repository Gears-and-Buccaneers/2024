package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.*;

public class SamKeyboard extends Keyboard implements Operator, Driver {
  public SamKeyboard(int port) {
    super(port);
  }

  @Override
  public Trigger intakePiece() {
    return ONE;
  }

  @Override
  public Trigger outtakeTopPiece() {
    return THREE;
  }

  @Override
  public Trigger outtakeMidPiece() {
    return TWO;
  }

  @Override
  public double getDrivetrainTranslationX() {
    return WandS.get();
  }

  @Override
  public double getDrivetrainTranslationY() {
    return DandA.get();
  }

  @Override
  public double getDrivetrainRotation() {
    return EandQ.get();
  }

  @Override
  public Trigger zeroGyro() {
    return ZERO;
  }

  @Override
  public Trigger setPose() {
    return NINE;
  }

  public Command rumble() {
    return null;
  }
}
