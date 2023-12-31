package frc.robot.Subsytems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSub extends SubsystemBase implements AutoCloseable {
  public final ArmRequirments arm;

  private final String simpleName = this.getClass().getSimpleName();

  public ArmSub(ArmRequirments arm) {
    this.arm = arm;
    arm.wristAngleSetpoint(Rotation2d.fromDegrees(240));
    arm.elevatorAngleSetpoint(Rotation2d.fromDegrees(30));
    arm.elevatorLengthSetpoint(Units.inchesToMeters(25));

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + arm.getClass().getSimpleName());

    this.arm.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, arm);

    arm.loadPreferences();
    arm.periodic();
  }

  @Override
  public void simulationPeriodic() {
  }

  // Commands ---------------------------------------------------------
  public Command IntakePosition() {
    return run(() -> {
      arm.wristAngleSetpoint(Rotation2d.fromDegrees(270));
      arm.elevatorAngleSetpoint(Rotation2d.fromDegrees(15));
      arm.elevatorLengthSetpoint(Units.inchesToMeters(35));
    })
        .handleInterrupt(
            () -> {
              arm.disable();
            });
  }

  public Command OutakeTopPositon() {
    return run(() -> {
      arm.wristAngleSetpoint(Rotation2d.fromDegrees(90));
      arm.elevatorAngleSetpoint(Rotation2d.fromDegrees(120));
      arm.elevatorLengthSetpoint(Units.inchesToMeters(55));
    })
        .handleInterrupt(
            () -> {
              arm.disable();
            });
  }

  public Command OutakeMidPositon() {
    return run(() -> {
      arm.wristAngleSetpoint(Rotation2d.fromDegrees(100));
      arm.elevatorAngleSetpoint(Rotation2d.fromDegrees(110));
      arm.elevatorLengthSetpoint(Units.inchesToMeters(35));
    })
        .handleInterrupt(
            () -> {
              arm.disable();
            });
  }

  public Command SafePositon() {
    return run(() -> {
      arm.wristAngleSetpoint(Rotation2d.fromDegrees(240));
      arm.elevatorAngleSetpoint(Rotation2d.fromDegrees(30));
      arm.elevatorLengthSetpoint(Units.inchesToMeters(25));
    })
        .handleInterrupt(
            () -> {
              arm.disable();
            });
  }

  public Command IntakePositionAuton() {
    return IntakePosition()
        .until(() -> {
          return arm.atSetpoint();
        });
  }

  public Command OutakeTopPositonAuton() {
    return OutakeTopPositon()
        .until(() -> {
          return arm.atSetpoint();
        });
  }

  public Command OutakeMidPositonAuton() {
    return OutakeMidPositon()
        .until(() -> {
          return arm.atSetpoint();
        });
  }

  // ---------------------------------------------------------
  @Override
  public void close() throws Exception {
    arm.close();
  }
}
