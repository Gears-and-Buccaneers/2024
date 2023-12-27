package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.sensor.proximitySwitch.ProximitySwitch;

import org.littletonrobotics.junction.Logger;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
  private final IntakeRequirments intakeIO;
  private final ProximitySwitch proximitySwitch;
  
  private final String simpleName = this.getClass().getSimpleName();

  public IntakeSub(IntakeRequirments intakeIO, ProximitySwitch proximitySwitch) {
    System.out.println("[Init] Creating " + simpleName + " w/ " + intakeIO.getClass().getSimpleName());

    this.intakeIO = intakeIO;

    this.proximitySwitch = proximitySwitch;

    intakeIO.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, intakeIO);
    Logger.processInputs(simpleName + "/proximitySwitch", proximitySwitch);

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
        .onlyIf(proximitySwitch::isOpen)
        .handleInterrupt(() -> {
          intakeIO.off();
        }).until(proximitySwitch::isClosed);
  }

  public Command ejectPice() {
    return run(
        () -> {
          intakeIO.setOutakeVoltage();
        })
        .onlyIf(proximitySwitch::isClosed)
        .handleInterrupt(() -> {
          intakeIO.off();
        })
        .until(proximitySwitch::isOpen);
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
    proximitySwitch.close();
  }
}
