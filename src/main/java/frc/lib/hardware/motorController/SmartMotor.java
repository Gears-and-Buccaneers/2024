package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.lib.hardware.sensor.encoders.Encoder;

//pid to controll percent output
public class SmartMotor implements Motor {
    private Motor motor;
    private Encoder encoder;

    private String logName;

    public SmartMotor(Motor motor, Encoder encoder, String logName) {
        this.motor = motor;
        this.logName = logName;
        this.encoder = encoder;
    }

    @Override
    public void runPercentOut(double num) {
        motor.runPercentOut(num);
    }

    @Override
    public void brakeMode(boolean enable) {
        motor.brakeMode(enable);
    }

    /** true inverts it */
    @Override
    public void setInverted(boolean enable) {
        motor.setInverted(enable);
    }

    public void disable() {
        motor.disable();
    }

    // ----------------------------------------------------------
    @Override
    public int getCanID() {
        return motor.getCanID();
    }

    @Override
    public void toLog(LogTable table) {
        Logger.processInputs(logName + "/encoder", encoder);
        Logger.processInputs(logName + "/Motor", motor);

    }

    @Override
    public boolean connected() {
        return true; // TODO probaly Should FIx
    }

    // Unit Testing
    @Override
    public void close() throws Exception {

    }
    

    // returns the rotation in degrees acounting for gearboxes and stuff
    // abstract double getRotation();

    /**
     * Inverted
     * Coast or break
     * max percent output/voltage
     * min percent output/voltage
     * 
     * PID
     * p
     * i
     * d
     * s
     * v
     * a
     * 
     * Motion magic
     * max accelratoin
     * max decelratoin
     * cruse velocity
     * 
     * feed forward (optonal)
     * 
     * 
     * can take a encoder
     * set gear ratio
     * set encoder counts per tick
     * 
     * 
     * controll modes
     * percent output
     * voltage
     * velocty
     * 
     */
}
