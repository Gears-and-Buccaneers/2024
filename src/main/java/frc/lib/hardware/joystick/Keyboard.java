package frc.lib.hardware.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Keyboard {
  private static final double deadband = 0.05;

  public final Trigger ONE,
      TWO,
      THREE,
      FOUR,
      FIVE,
      SIX,
      SEVEN,
      EIGHT,
      NINE,
      ZERO;
  public final JoystickAxis WandS, DandA, EandQ;

  public Keyboard(int port) {
    Joystick joystick = new Joystick(port);

    WandS = new JoystickAxis(joystick, 0, deadband, 2);
    DandA = new JoystickAxis(joystick, 1, deadband, 2);
    EandQ = new JoystickAxis(joystick, 2, deadband, 2);

    ONE = new JoystickButton(joystick, 1);
    TWO = new JoystickButton(joystick, 2);
    THREE = new JoystickButton(joystick, 3);
    FOUR = new JoystickButton(joystick, 4);
    FIVE = new JoystickButton(joystick, 5);
    SIX = new JoystickButton(joystick, 6);
    SEVEN = new JoystickButton(joystick, 7);
    EIGHT = new JoystickButton(joystick, 8);
    NINE = new JoystickButton(joystick, 9);
    ZERO = new JoystickButton(joystick, 10);
  }
}
