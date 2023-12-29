package frc.lib.hardware.Motors.MotorControlers;

import frc.lib.hardware.HardwareRequirments;

public interface MotorController extends HardwareRequirments {

  //kMotor(Tallon_SRX, CIM, id).invert().pid(new PIDConfig(asdlhjasdfhas)).encoder(new kEncoder(encodertyoe, id))
  void runPercentOut(double num);

  void brakeMode(boolean enable);

  void setInverted(boolean enable);

  int getCanID();

  void disable();
}
