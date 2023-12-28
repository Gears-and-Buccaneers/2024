package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final String simpleName = this.getClass().getSimpleName();

  private final IntakeRequirments intakeIO;

  public IntakeSub(IntakeRequirments intakeIO) {
    this.intakeIO = intakeIO;

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + intakeIO.getClass().getSimpleName());

    this.intakeIO.setLogName(simpleName);

    intakeIO.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, intakeIO);

    intakeIO.loadPreferences();
    intakeIO.periodic();
  }

  @Override
  public void simulationPeriodic() {

  }

  public Command intakePice() {
    return run(
        () -> {
          intakeIO.setIntakeVoltage();
        })
        // .onlyIf(intakeIO::isOpen)
        .handleInterrupt(() -> {
          intakeIO.off();
        });
    // .until(intakeIO::isClosed);
  }

  public Command ejectPice() {
    return run(
        () -> {
          intakeIO.setOutakeVoltage();
        })
        // .onlyIf(intakeIO::isClosed)
        .handleInterrupt(() -> {
          intakeIO.off();
        });
    // .until(intakeIO::isOpen);
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
