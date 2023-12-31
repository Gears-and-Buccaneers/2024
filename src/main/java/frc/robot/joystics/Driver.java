package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Driver {
  double getDrivtrainTranslationX();

  double getDrivtrainTranslationY();

  double getDrivtrainRotation();

  Trigger zeroGyro();

  Command rumble();
}
