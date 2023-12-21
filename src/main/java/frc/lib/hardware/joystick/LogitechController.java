package frc.lib.hardware.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechController {
    private static final double deadband = 0.05;

    public final Trigger A, B, X, Y, LB, RB, BACK, START, LT, RT, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT,
            LEFT,
            UP_LEFT, CENTER;
    public final JoystickAxis LS_X, LS_Y, RS_X, RS_Y, LT_S, RT_S;

    public LogitechController(int port) {
        Joystick joystick = new Joystick(port);

        LS_X = new JoystickAxis(joystick, 0, deadband, 2);
        LS_Y = new JoystickAxis(joystick, 1, deadband, 2);
        RS_X = new JoystickAxis(joystick, 4, deadband, 2);
        RS_Y = new JoystickAxis(joystick, 5, deadband, 2);

        LT_S = new JoystickAxis(joystick, 2, deadband, 3);
        RT_S = new JoystickAxis(joystick, 3, deadband, 3);

        A = new JoystickButton(joystick, 1);
        B = new JoystickButton(joystick, 2);
        X = new JoystickButton(joystick, 3);
        Y = new JoystickButton(joystick, 4);
        LB = new JoystickButton(joystick, 5);
        RB = new JoystickButton(joystick, 6);
        BACK = new JoystickButton(joystick, 7);
        START = new JoystickButton(joystick, 8);
        LT = new JoystickButton(joystick, 9);
        RT = new JoystickButton(joystick, 10);

        UP = new Trigger(() -> joystick.getPOV() == 0);
        UP_RIGHT = new Trigger(() -> joystick.getPOV() == 45);
        RIGHT = new Trigger(() -> joystick.getPOV() == 90);
        DOWN_RIGHT = new Trigger(() -> joystick.getPOV() == 135);
        DOWN = new Trigger(() -> joystick.getPOV() == 180);
        DOWN_LEFT = new Trigger(() -> joystick.getPOV() == 225);
        LEFT = new Trigger(() -> joystick.getPOV() == 270);
        UP_LEFT = new Trigger(() -> joystick.getPOV() == 315);
        CENTER = new Trigger(() -> joystick.getPOV() == -1);
    }

}
