package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final IntakeRequirments intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSub(IntakeRequirments intakeIO) {
    System.out.println("[Init] Creating Intake w/ " + intakeIO.getClass().getSimpleName());
    this.intakeIO = intakeIO;

    intakeIO.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public IntakeIOInputsAutoLogged getInputs() {
    return inputs;
  }

  public Command intakePice() {
    return run(() -> {
          intakeIO.setVoltage(6);
        })
        .handleInterrupt(
            () -> {
              intakeIO.setVoltage(0);
            });
  }

  /**
   * @return A command that sets the voltage of the Intake to 6 when executed, and sets it to 0 when
   *     interrupted.
   */
  public Command ejectPice() {
    return run(() -> {
          intakeIO.setVoltage(6);
        })
        .handleInterrupt(
            () -> {
              intakeIO.setVoltage(0);
            });
  }

  public Command stopIntake() {
    return run(
        () -> {
          intakeIO.setVoltage(0);
        });
  }

  @Override
  public void close() throws Exception {
    intakeIO.close();
  }
}