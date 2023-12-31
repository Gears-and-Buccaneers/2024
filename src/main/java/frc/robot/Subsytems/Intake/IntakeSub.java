package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final String simpleName = this.getClass().getSimpleName();

  private final IntakeRequirments intake;

  public IntakeSub(IntakeRequirments intake) {
    this.intake = intake;

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + intake.getClass().getSimpleName());

    intake.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, intake);

    intake.periodic();
  }

  @Override
  public void simulationPeriodic() {}

  // Commands ------------------------------
  public Command intakePice() {
    return run(() -> {
          intake.setIntakeSpeed();
        })
        // .onlyIf(intakeIO::isOpen)
        .handleInterrupt(
            () -> {
              intake.disable();
            });
    // .until(intakeIO::isClosed);
  }

  public Command ejectPice() {
    return run(() -> {
          intake.setOutakeSpeed();
        })
        // .onlyIf(intakeIO::isClosed)
        .handleInterrupt(
            () -> {
              intake.disable();
            });
    // .until(intakeIO::isOpen);
  }

  public Command stopIntake() {
    return run(
        () -> {
          intake.disable();
        });
  }

  // Closing ------------------------------
  @Override
  public void close() throws Exception {
    intake.close();
  }
}
