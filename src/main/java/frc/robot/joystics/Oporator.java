package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Oporator {

  /** intake moter set to intake 50% */
  Trigger intakePice();

  /** intake moter set to outtake 50% */
  Trigger OuttakePice();

  double getDrivtrainTranslationX();

  double getDrivtrainTranslationY();

  double getDrivtrainRotation();
}
