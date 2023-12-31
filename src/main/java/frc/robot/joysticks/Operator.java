package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Operator {

  /** intake moter set to intake 50% */
  Trigger intakePiece();

  /** intake moter set to outtake 50% */
  Trigger outtakeTopPiece();

  Trigger outtakeMidPiece();

  Trigger setPose();
}
