package frc.lib.hardware.motorController;

public interface SmartMotor extends Motor{
    //returns the rotation in degrees acounting for gearboxes and stuff
    double getRotation();
}
