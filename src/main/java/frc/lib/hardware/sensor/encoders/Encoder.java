package frc.lib.hardware.sensor.encoders;

import frc.lib.hardware.HardwareRequirments;

public interface Encoder extends HardwareRequirments {

    double getPositoin();

    double getVelocity();

    void setGearRatio(double gearRatio);
}
