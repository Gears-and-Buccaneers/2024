package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.joystick.LogitechController;

public class Logictech extends LogitechController implements Oporator {
    public Logictech(int port) {
        super(port);
        // TODO Auto-generated constructor stub
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
