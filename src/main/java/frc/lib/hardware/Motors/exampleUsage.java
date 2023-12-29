package frc.lib.hardware.Motors;

import frc.lib.hardware.Motors.PID.*;
import frc.lib.hardware.sensor.encoders.REVBoreEncoder;

public class exampleUsage {
  private Motor motor1;

  private Motor motor2;

  public exampleUsage() {
    motor1 = new Motor(Motor.ControllerType.TallonSRX, 10, Motor.Type.VP775);

    motor2 = new Motor(Motor.ControllerType.TallonSRX, 11, Motor.Type.VP775);
    motor2
        .addEncoder(new REVBoreEncoder())
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs());

    motor1.inverted(true);
    motor2.inverted(false);
  }

  public void setMotorAngle() {}
}
