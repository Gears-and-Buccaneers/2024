package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.joystics.Driver;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable {
  public final DrivetrainRequirments drivetrain;

  private final String simpleName = this.getClass().getSimpleName();

  public DrivetrainSub(DrivetrainRequirments drivetrain) {
    this.drivetrain = drivetrain;

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + this.drivetrain.getClass().getSimpleName());

    this.drivetrain.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, drivetrain);

    drivetrain.loadPreferences();
    drivetrain.periodic();
  }

  @Override
  public void simulationPeriodic() {}

  // Commands ---------------------------------------------------------
  public Command drive(Driver controler) {
    return run(
        () -> {
          drivetrain.setChassisSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  controler.getDrivtrainTranslationX() * 4,
                  controler.getDrivtrainTranslationY() * 4,
                  controler.getDrivtrainRotation() * 2,
                  drivetrain.getAngle()));
        });
  }

  // ---------------------------------------------------------
  @Override
  public void close() throws Exception {
    drivetrain.close();
  }
}
