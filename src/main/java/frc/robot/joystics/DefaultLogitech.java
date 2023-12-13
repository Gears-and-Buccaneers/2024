package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class DefaultLogitech extends LogitechController implements Oporator {
    public DefaultLogitech(int port) {
        super(port);
    }

    @Override
    public Trigger deploy() {
        return X;
    }

    @Override
    public Trigger retract() {
        return B;
    }

    @Override
    public Trigger intake() {
        return A;
    }

    @Override
    public Trigger outtake() {
        return Y;
    }

}
