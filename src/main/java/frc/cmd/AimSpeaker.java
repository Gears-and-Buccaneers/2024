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
import edu.wpi.first.wpilibj2.command.Command;
import frc.Robot;
import frc.system.Pivot;
import frc.system.Shooter;
import frc.system.Swerve;

public class AimSpeaker extends Command {
    private final String simpleName = this.getClass().getSimpleName();

    // Subsystems
    private final Swerve drivetrain;
    private final Pivot pivot;
    private final Shooter shooter;

    // Network
    /** the sub table where all logging for Shooter should go */
    private final NetworkTable autoAimTable;

    /**
     * the offset for the rotation of the pivot positive shoots higher/lower
     */
    private final DoubleSubscriber rotationFudge;
    /** offest the rotation of the robot RADS Positive rottes x */
    private final DoubleSubscriber yawFudge;
    /** the offset for rpm of the shooter speed */
    private final DoubleSubscriber shooterFudge;

    private double yaw;
    private double distance;
    private double pitch;
    private double rotations;
    private double shooterSpeed;

    private final DoublePublisher yawPub;
    private final DoublePublisher distancePub;
    private final DoublePublisher rotationsPub;
    private final DoublePublisher shooterSpeedPub;

    // Vars
    private final Translation3d speakerPosition;
    // private DoubleSubscriber fudgeFactor;

    public AimSpeaker(Swerve drivetrain, Pivot pivot, Shooter shooter) {
        // Network tables
        this.autoAimTable = NetworkTableInstance.getDefault().getTable("Commands").getSubTable(simpleName);

        this.autoAimTable.getDoubleTopic("RotationFudge").publish();
        rotationFudge = autoAimTable.getDoubleTopic("RotationFudge").subscribe(0);
        this.autoAimTable.getDoubleTopic("yawFudge").publish();
        yawFudge = autoAimTable.getDoubleTopic("yawFudge").subscribe(0);
        this.autoAimTable.getDoubleTopic("ShooterFudge").publish();
        shooterFudge = autoAimTable.getDoubleTopic("ShooterFudge").subscribe(0);

        yawPub = this.autoAimTable.getDoubleTopic("yaw").publish();
        distancePub = this.autoAimTable.getDoubleTopic("distance").publish();
        rotationsPub = this.autoAimTable.getDoubleTopic("rotations").publish();
        shooterSpeedPub = this.autoAimTable.getDoubleTopic("shooterSpeed").publish();

        // Subsystems
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.shooter = shooter;

        // Vars
        speakerPosition = Robot.isRedAlliance
                ?
                // Red speaker
                new Translation3d(16.31, 5.55, 2.06)
                // Blue speaker
                : new Translation3d(0.24, 5.55, 2.06);

        addRequirements(pivot, shooter);
    }

    public void doMath() {
        // Rose to speaker
        Pose2d mechanismPose = drivetrain.pose()
                .plus(new Transform2d(pivot.origin.getX(), pivot.origin.getY(),
                        new Rotation2d()));
        Translation3d mechanism = new Translation3d(mechanismPose.getX(), mechanismPose.getY(),
                pivot.origin.getZ());
        Translation3d vectorToSpeaker = speakerPosition.minus(mechanism);

        // Drivetrain yaw
        yaw = Math.atan2(vectorToSpeaker.getY(), vectorToSpeaker.getX());
        yaw += 0.175;
        yaw += yawFudge.get(0);

        // pivot Angle
        distance = vectorToSpeaker.getNorm();
        pitch = Math.asin(vectorToSpeaker.getZ() / distance);
        rotations = Units
                .radiansToRotations(pivot.armOffsetRad + Math.asin(pivot.exitDistance /
                        distance) - pitch)
                + rotationFudge.get(0);
        rotations += -0.005;
        rotations += -0.00125 * distance;
        //might want to add + .02

        // Shooter Speed
        shooterSpeed = 4000;
        shooterSpeed += 100 * distance;
        shooterSpeed += shooterFudge.get(0);

        if (shooterSpeed > 5000)
            shooterSpeed = 5000;
    }

    @Override
    public void execute() {
        doMath();
        log();

        pivot.MMPositionCtrl(rotations);
        drivetrain.setRotationOverride(Rotation2d.fromRadians(yaw));
        shooter.VelocityOpenLoop(true, shooterSpeed);
        // alongWith(pivot.MMPositionCtrl(rotations1)); // Testing this is this does not
        // work the above line works
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setRotationOverride(null);
        // shooter.disable();
    }

    public void log() {
        // drivetrain.log();
        // pivot.log();

        yawPub.set(yaw);
        rotationsPub.set(rotations);
        distancePub.set(distance);
        shooterSpeedPub.set(shooterSpeed);

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
