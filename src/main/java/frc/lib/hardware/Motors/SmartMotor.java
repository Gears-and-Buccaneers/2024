package frc.lib.hardware.Motors;

import frc.lib.hardware.Motors.MotorControlers.MotorController;
import frc.lib.hardware.Motors.PID.*;
import frc.lib.hardware.sensor.encoders.Encoder;

public class SmartMotor extends Motor {

    public SmartMotor(MotorController motorController, MotorType motor, Encoder encoder) {
        super(motorController, motor);
    }

    public SmartMotor inverted(boolean enable) {
        super.inverted(enable);

        return this;
    }

    public SmartMotor pidConfigs(PIDConfigs PIDConfigs) {

        return this;
    }

    public SmartMotor EncoderConfigs(EncoderConfigs encoderConfigs) {

        return this;
    }

    // kMotor(Tallon_SRX, CIM, id).invert().pid(new
    // PIDConfig(asdlhjasdfhas)).encoder(new kEncoder(encodertyoe, id))
}
