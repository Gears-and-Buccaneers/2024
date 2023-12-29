package frc.lib.hardware.Motors;

import frc.lib.hardware.Motors.Motor.MotorType;
import frc.lib.hardware.Motors.MotorControlers.*;
import frc.lib.hardware.Motors.PID.EncoderConfigs;
import frc.lib.hardware.Motors.PID.PIDConfigs;
import frc.lib.hardware.sensor.encoders.REVBoreEncoder;

public class exampleUsage {
    private Motor motor;
    private SmartMotor motor2;

    public exampleUsage() {
        motor = new Motor(new Talon_SRX(1), MotorType.VP775);
        motor.inverted(true);

        motor2 = new SmartMotor(new Talon_SRX(0), MotorType.Falcon500, new REVBoreEncoder()).inverted(false)
                .pidConfigs(new PIDConfigs()).EncoderConfigs(new EncoderConfigs());
    }

    public void setMotorAngle() {

    }

    // kMotor(Tallon_SRX, CIM, id).invert().pid(new
    // PIDConfig(asdlhjasdfhas)).encoder(new kEncoder(encodertyoe, id))
}
