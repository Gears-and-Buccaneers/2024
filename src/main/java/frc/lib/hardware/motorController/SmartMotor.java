package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.hardware.sensor.encoders.Encoder;

public class SmartMotor implements Motor {
    

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
