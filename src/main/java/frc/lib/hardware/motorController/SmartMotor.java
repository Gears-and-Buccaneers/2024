package frc.lib.hardware.motorController;


import frc.lib.hardware.sensor.encoders.Encoder;

public abstract class SmartMotor{

    public SmartMotor(int id, Encoder encoder) {

    }
    //returns the rotation in degrees acounting for gearboxes and stuff
    abstract double getRotation();
}
