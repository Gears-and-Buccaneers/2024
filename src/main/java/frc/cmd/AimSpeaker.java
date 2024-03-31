package frc.cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Pivot;
import frc.system.Swerve;

public class AimSpeaker extends Command {
    private final String simpleName = this.getClass().getSimpleName();

    // Subsystems
    private final Swerve drivetrain;
    private final Pivot pivot;

    // Network
    /** the sub table where all logging for Shooter should go */
    private final NetworkTable autoAimTable;

    /**
     * the offset for the rotation of the pivot positive shoots higher/lower
     */// TODO: check
    private final DoubleSubscriber rotationFudge;
    /** offest the rotation of the robot RADS Positive rottes x */
    private final DoubleSubscriber yawFudge;

    private final DoublePublisher yaw;
    private final DoublePublisher distance;
    private final DoublePublisher rotations;

    // Vars
    private final Translation3d speakerPosition;
    // private DoubleSubscriber fudeFactor;

    public AimSpeaker(Swerve drivetrain, Pivot pivot) {
        // Network tables
        this.autoAimTable = NetworkTableInstance.getDefault().getTable("Commands").getSubTable(simpleName);

        this.autoAimTable.getDoubleTopic("RotationFudge").publish();
        rotationFudge = autoAimTable.getDoubleTopic("RotationFudge").subscribe(0);
        this.autoAimTable.getDoubleTopic("yawFudge").publish();
        yawFudge = autoAimTable.getDoubleTopic("yawFudge").subscribe(0);

        yaw = this.autoAimTable.getDoubleTopic("yaw").publish();
        distance = this.autoAimTable.getDoubleTopic("distance").publish();
        rotations = this.autoAimTable.getDoubleTopic("rotations").publish();

        // Subsystems
        this.drivetrain = drivetrain;
        this.pivot = pivot;

        // Vars
        speakerPosition = DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent()
                ?
                // Red speaker
                new Translation3d(16.31, 5.55, 3.06)
                // Blue speaker
                : new Translation3d(0.24, 5.55, 3.06);

        addRequirements(pivot);
    }

    private double yaw1;
    private double distance1;
    private double pitch1;
    private double rotations1;

    private final double a = -0.260927;
    private final double b = 0.242026;

    public void doMath() {
        Pose2d mechanismPose = drivetrain.pose()
                .plus(new Transform2d(pivot.origin.getX(), pivot.origin.getY(),
                        new Rotation2d()));
        Translation3d mechanism = new Translation3d(mechanismPose.getX(), mechanismPose.getY(),
                pivot.origin.getZ());
        Translation3d vectorToSpeaker = speakerPosition.minus(mechanism);

        // Drivetrain yaw
        yaw1 = Math.atan2(vectorToSpeaker.getY(), vectorToSpeaker.getX());
        yaw1 += yawFudge.get(0) + .2;

        distance1 = vectorToSpeaker.getNorm();
        pitch1 = Math.asin(vectorToSpeaker.getZ() / distance1);
        rotations1 = Units
                .radiansToRotations(pivot.armOffsetRad + Math.asin(pivot.exitDistance /
                        distance1) - pitch1)
                + rotationFudge.get(0);
        // rotations1 = rotationFudge.get(0);
        // rotations1 = a + b * Math.log(distance1) + rotationFudge.get(0);

    }

    @Override
    public void execute() {
        doMath();
        log();

        drivetrain.setRotationOverride(Rotation2d.fromRadians(yaw1));
        pivot.setPosition(rotations1);
        // alongWith(pivot.MMPositionCtrl(rotations1)); // Testing this is this does not
        // work the above line works
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setRotationOverride(null);
        pivot.toIntake();// TODO: do we want this here
    }

    public void log() {
        drivetrain.log();
        pivot.log();

        yaw.set(yaw1);
        rotations.set(distance1);
        distance.set(distance1);

    }

    // public void close() {
    // // Subsystems
    // drivetrain.close();
    // pivot.close();

    // // // Network Table
    // // rotationFudge.close();
    // // yawFudge.close();

    // // yaw.close();
    // // rotations.close();
    // // distance.close();
    // }
}
