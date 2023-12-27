package frc.lib.hardware.sensor.proximitySwitch;

import edu.wpi.first.wpilibj.DigitalInput;

public class Huchoo implements ProximitySwitch {
    private DigitalInput huchoo;
    // private int DIOChannel;

    /**
     * @param DIOChannel the DIO channel for the digital input 0-9 are on-board
     */
    public Huchoo(int DIOChannel) {
        // this.DIOChannel = DIOChannel;
        huchoo = new DigitalInput(DIOChannel);
        System.out.println("[init] new " + this.getClass().getSimpleName() + " on DIO port " + DIOChannel);
    }

    @Override
    public boolean get() {
        return huchoo.get();
    }

    @Override
    public boolean connected() {
        return true;
    }

    @Override
    public void close() throws Exception {
        huchoo.close();
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }

    @Override
    public double getPeriod() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPeriod'");
    }
}
