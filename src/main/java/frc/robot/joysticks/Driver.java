package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Driver {
  double getDrivetrainTranslationX();

  double getDrivetrainTranslationY();

  double getDrivetrainRotation();

  Trigger zeroGyro();

  Command rumble();
}
