package frc.lib.hardware.sensor.proximitySwitch;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.wpilibj.DigitalInput;

public class Huchoo implements ProximitySwitch {
    private DigitalInput huchoo;
    private int DIOChannel;
    // private int DIOChannel;

    /**
     * @param DIOChannel the DIO channel for the digital input 0-9 are on-board
     */
    public Huchoo(int DIOChannel) {
        this.DIOChannel = DIOChannel;
        huchoo = new DigitalInput(DIOChannel);
        System.out.println("[init] new " + this.getClass().getSimpleName() + " on DIO port " + DIOChannel);
    }

    @Override
    public boolean get() {
        return huchoo.get();
    }

    public int getDIOChannel() {
        return DIOChannel;
    }

    // ----------------------------------------------------------
    @Override
    public void toLog(LogTable table) {
        table.put("Open", get());
    }

    @Override
    public boolean connected() {
        return true; // TODO probaly Should FIx
    }

    // Unit Testing
    @Override
    public void close() throws Exception {
        huchoo.close();
    }
}
