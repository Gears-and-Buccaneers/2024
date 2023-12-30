package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.joystics.Driver;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable {
  public final DrivetrainReq drivetrain;

  private final String simpleName = this.getClass().getSimpleName();

  public DrivetrainSub(DrivetrainReq drivetrain) {
    this.drivetrain = drivetrain;
    AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drivetrain::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        maxSpeed, // Max module speed, in m/s
                        drivetrainRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                this // Reference to this subsystem to set requirements
        );

    System.out.println("[Init] Creating " + simpleName + " with:");
    System.out.println("\t" + this.drivetrain.getClass().getSimpleName());

    this.drivetrain.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    Logger.processInputs(simpleName, drivetrain);

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
                  controler.getDrivtrainTranslationY() * -4,
                  controler.getDrivtrainRotation() * 6 * 4,
                  drivetrain.getAngle()));
        });
  }

  // ---------------------------------------------------------
  @Override
  public void close() throws Exception {
    drivetrain.close();
  }
}
