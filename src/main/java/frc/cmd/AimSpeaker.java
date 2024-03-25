package frc.cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Pivot;
import frc.system.Swerve;

public class AimSpeaker extends Command {
    private final Swerve drivetrain;
    private final Pivot pivot;

    private final Translation3d speakerPosition;

    public AimSpeaker(Swerve drivetrain, Pivot pivot) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;

        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(16.31, 5.55, 2.06)
                // Blue speaker
                : new Translation3d(0.24, 5.55, 2.06);

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        Pose2d mechanismPose = drivetrain.pose()
                .plus(new Transform2d(pivot.origin.getX(), pivot.origin.getY(),
                        new Rotation2d()));
        Translation3d mechanism = new Translation3d(mechanismPose.getX(), mechanismPose.getY(),
                pivot.origin.getZ());
        Translation3d vectorToSpeaker = speakerPosition.minus(mechanism);

        // Drivetrain yaw
        double yaw = Math.atan2(vectorToSpeaker.getY(), vectorToSpeaker.getX());

        drivetrain.setRotationOverride(new Rotation2d(yaw));

        // Pivot pitch
        double distance = vectorToSpeaker.getNorm();
        double pitch = Math.asin(vectorToSpeaker.getZ() / distance);

        pivot.aimAt(distance, pitch);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setRotationOverride(null);
        // pivot.intake();// TODO: do we want this here
    }
}
