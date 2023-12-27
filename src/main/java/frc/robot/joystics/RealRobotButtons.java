package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class RealRobotButtons implements RobotButtons {

    private ProximitySwitch button1;
    private ProximitySwitch button2;

    public RealRobotButtons() {
        button1 = new LimitSwitch(3);
        button2 = new LimitSwitch(2);
    }

    @Override
    public Trigger toggleBreakMode() {
        return button1.trigger();

    }

    @Override
    public Trigger zeroSensors() {
        return button2.trigger();
    }

    @Override
    public void close() throws Exception {
        button1.close();
        button2.close();
    }

}
