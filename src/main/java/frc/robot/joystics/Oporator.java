package frc.robot.joystics;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Oporator {
    /** deploys the intake */
    Trigger deploy();

    /** retracts the intake */
    Trigger retract();

    /** intake moter set to intake 50% */
    Trigger intake();

    /** intake moter set to outtake 50% */
    Trigger outtake();
}
