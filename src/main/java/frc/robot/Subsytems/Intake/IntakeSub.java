package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.sensor.proximitySwitch.ProximitySwitch;

import org.littletonrobotics.junction.Logger;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final IntakeRequirments intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final ProximitySwitch sensor;

  public IntakeSub(IntakeRequirments intakeIO, ProximitySwitch proximitySwitch) {
    System.out.println("[Init] Creating " +
        this.getClass().getSimpleName() + " w/ " +
        intakeIO.getClass().getSimpleName());

    this.intakeIO = intakeIO;

    sensor = proximitySwitch;

    intakeIO.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs(this.getClass().getSimpleName(), inputs);
    Logger.processInputs(this.getClass().getSimpleName() + "/proximitySwitch", sensor);

    intakeIO.loadPreferences();
  }

  @Override
  public void simulationPeriodic() {

  }

  public IntakeIOInputsAutoLogged getInputs() {
    return inputs;
  }

  public Command intakePice() {
    return run(
        () -> {
          intakeIO.setIntakeVoltage();
        })
        .onlyIf(sensor::isOpen)
        .handleInterrupt(() -> {
          intakeIO.off();
        });
  }

  public Command ejectPice() {
    return run(
        () -> {
          intakeIO.setOutakeVoltage();
        })
        .onlyIf(sensor::isClosed)
        .handleInterrupt(() -> {
          intakeIO.off();
        });
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
    sensor.close();
  }
}
