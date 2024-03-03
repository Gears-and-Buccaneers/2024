package frc.system.drivetrain;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Drivetrain;
import frc.system.Vision.Measurement;

public class SwerveDrive extends SwerveDrivetrain implements Drivetrain {
    private final String simpleName = this.getClass().getSimpleName();

    // Network
    private NetworkTable table;

    // vars
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public double kSpeedAt12VoltsMps = 3.92;
    public double MaxAngularRate = Units.degreesToRadians(520);

    private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();

    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.table = NetworkTableInstance.getDefault().getTable("Drivetrain");

        registerTelemetry((s) -> state.set(s));

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }

        System.out.println("[Init] Creating " + simpleName + " with:");

        this.log();
    }

    // SwerveRequest
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kSpeedAt12VoltsMps * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveAt = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(kSpeedAt12VoltsMps * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake xState = new SwerveRequest.SwerveDriveBrake();

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // Commands
    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public Command drive(double VelocityX, double VelocityY, double RotationalRate) {
        return applyRequest(() -> drive
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withRotationalRate(RotationalRate));
    }

    @Override
    public Command xState() {
        return applyRequest(() -> xState);
    }

    @Override
    public Command zeroGyro() {
        return runOnce(() -> this.seedFieldRelative());
    }

    @Override
    public Command driveFacing(double VelocityX, double VelocityY, Rotation2d TargetDirection) {
        return applyRequest(() -> driveAt
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withTargetDirection(TargetDirection));
    }

    @Override
    public Command driveTo(Pose2d position, double velocity) {
        PathConstraints constraints = new PathConstraints(velocity, 4, Units.degreesToRadians(520),
                Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(position, constraints, 0, 0);
    }

    @Override
    public Command DriveToThenPath(PathPlannerPath path, double velocity) {
        PathConstraints constraints = new PathConstraints(velocity, 4, Units.degreesToRadians(520),
                Units.degreesToRadians(720));

        return AutoBuilder.pathfindThenFollowPath(path, constraints, 0);
    }

    // Autos
    @Override
    public SendableChooser<Command> getAutoPaths() {
        return AutoBuilder.buildAutoChooser();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    private Optional<Rotation2d> getRotationTargetOverride() {
        // // Some condition that should decide if we want to override rotation
        // if (Limelight.hasGamePieceTarget()) {
        // // Return an optional containing the rotation override (this should be a
        // field
        // // relative rotation)
        // return Optional.of(Limelight.getRobotToGamePieceRotation());
        // } else {
        // // return an empty optional when we don't want to override the path's
        // rotation
        return Optional.empty();
        // }
    }

    // Odometry
    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public void reset(Pose2d pose) {
        seedFieldRelative(pose);
    }

    @Override
    public void accept(Measurement t) {
        if (t.stdDev() != null) {
            addVisionMeasurement(t.pose().toPose2d(), t.timestamp(), t.stdDev());
        } else {
            addVisionMeasurement(t.pose().toPose2d(), t.timestamp());
        }
    }

    // Logging
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final DoublePublisher velocityX = table.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = table.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = table.getDoubleTopic("Speed").publish();

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    @Override
    public void log() {
        /* Telemeterize the pose */
        Pose2d pose = getPose();
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
    }
}
