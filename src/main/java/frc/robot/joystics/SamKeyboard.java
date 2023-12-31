package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.*;

public class SamKeyboard extends Keyboard implements Oporator, Driver {
  public SamKeyboard(int port) {
    super(port);
  }

  @Override
  public Trigger intakePice() {
    return ONE;
  }

  @Override
  public Trigger OuttakeTopPice() {
    return THREE;
  }

  @Override
  public Trigger OuttakeMidPice() {
    return TWO;
  }

  @Override
  public double getDrivtrainTranslationX() {
    return WandS.get();
  }

  @Override
  public double getDrivtrainTranslationY() {
    return DandA.get();
  }

  @Override
  public double getDrivtrainRotation() {
    return EandQ.get();
  }
}
