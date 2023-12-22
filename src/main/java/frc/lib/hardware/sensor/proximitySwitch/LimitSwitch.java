package frc.lib.hardware.sensor.proximitySwitch;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch implements ProximitySwitch {
    private DigitalInput limitSwitch;

    /**
     * @param DIOChannel the DIO channel for the digital input 0-9 are on-board
     */
    public LimitSwitch(int DIOChannel) {
        limitSwitch = new DigitalInput(DIOChannel);
    }

    @Override
    public boolean get() {
        return limitSwitch.get();
    }

}
