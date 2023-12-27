package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final String simpleName = this.getClass().getSimpleName();

  private final IntakeRequirments intakeIO;

  public IntakeSub(IntakeRequirments intakeIO) {
    System.out.println("[Init] Creating " + simpleName + " w/ " + intakeIO.getClass().getSimpleName());

    this.intakeIO = intakeIO;

    intakeIO.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, intakeIO);

    intakeIO.loadPreferences();
  }

  @Override
  public void simulationPeriodic() {

  }

  public Command intakePice() {
    return run(
        () -> {
          intakeIO.setIntakeVoltage();
        })
        .onlyIf(intakeIO.getSwitch()::isOpen)
        .handleInterrupt(() -> {
          intakeIO.off();
        }).until(intakeIO.getSwitch()::isClosed);
  }

  public Command ejectPice() {
    return run(
        () -> {
          intakeIO.setOutakeVoltage();
        })
        .onlyIf(intakeIO.getSwitch()::isClosed)
        .handleInterrupt(() -> {
          intakeIO.off();
        })
        .until(intakeIO.getSwitch()::isOpen);
  }

  public Command stopIntake() {
    return run(
        () -> {
          intakeIO.off();
        });
  }

  @Override
  public void close() throws Exception {
    intakeIO.close();
  }
}
