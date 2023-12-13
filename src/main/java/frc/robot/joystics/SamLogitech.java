package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class SamLogitech extends LogitechController implements Oporator {
    public SamLogitech(int port) {
        super(port);
    }

    @Override
    public Trigger deploy() {
        return A;
    }

    @Override
    public Trigger retract() {
        return B;
    }

    @Override
    public Trigger intake() {
        return X;
    }

    @Override
    public Trigger outtake() {
        return Y;
    }

}
