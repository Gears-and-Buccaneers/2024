package frc.cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Pivot;
import frc.system.Swerve;

public class AimSpeaker extends Command {
    // Subsystems
    private final Swerve drivetrain;
    private final Pivot pivot;

    private final Translation3d speakerPosition;
    private DoubleSubscriber fudeFactor;

    public AimSpeaker(Swerve drivetrain, Pivot pivot) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("FudgeValue");
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(16.31, 5.55, 3.06)
                // Blue speaker
                : new Translation3d(0.24, 5.55, 3.06);
        table.getDoubleTopic("fudeFactorYaw").publish();
        fudeFactor = table.getDoubleTopic("fudeFactorYaw").subscribe(0);

        addRequirements(pivot);
    }

    public void doMath() {
        Pose2d mechanismPose = drivetrain.pose()
                .plus(new Transform2d(pivot.origin.getX(), pivot.origin.getY(),
                        new Rotation2d()));
        Translation3d mechanism = new Translation3d(mechanismPose.getX(), mechanismPose.getY(),
                pivot.origin.getZ());
        Translation3d vectorToSpeaker = speakerPosition.minus(mechanism);

        // Drivetrain yaw
        double yaw = Math.atan2(vectorToSpeaker.getY(), vectorToSpeaker.getX());
        yaw += fudeFactor.get(0);
        SmartDashboard.putNumber("yaw", yaw);

        double distance = vectorToSpeaker.getNorm();
        double pitch = Math.asin(vectorToSpeaker.getZ() / distance);
        double rotations = Units.radiansToRotations(armOffsetRad + Math.asin(exitDistance / distance) - pitch);

        SmartDashboard.putNumber("rotations", rotations);
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("pitch", pitch);

    }

    @Override
    public void execute() {
        drivetrain.setRotationOverride(Rotation2d.fromRadians(yaw));
        // pivot.setSetpoint(rotations);
        alongWith(pivot.MMPositionCtrl(rotations)); // Testing this is this does not work the above line works
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setRotationOverride(null);
        pivot.toIntake();// TODO: do we want this here
    }
}
