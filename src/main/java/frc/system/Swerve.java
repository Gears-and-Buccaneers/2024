package frc.system;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.config.SwerveConfig;

public class Swerve extends SwerveDrivetrain implements LoggedSubsystems {
    private final String simpleName = this.getClass().getSimpleName();

    // Control Modes
    private final SwerveDriveBrake cachedBrake = new SwerveDriveBrake();
    private final FieldCentric cachedFieldCentric = new FieldCentric();
    private final FieldCentricFacingAngle cachedFieldCentricFacing = new FieldCentricFacingAngle();
    private final ApplyChassisSpeeds AutoRequest = new ApplyChassisSpeeds();

    // Network
    private final NetworkTable Table;
    private final StructPublisher<Pose2d> ntPose2d;
    private final StructPublisher<Pose3d> ntPose3d;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleState;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleTarget;
    private final StructPublisher<Pose2d> visionPose2d;

    // vars
    private final PathConstraints ppConstraints;
    private final Rotation2d autoAimDeadband = Rotation2d.fromDegrees(5);

    private boolean hasPose = false;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();

    /**
     * The rotation override for the drivetrain. Set to null if there is no
     * override.
     */
    private Rotation2d rotationOverride;

    public Swerve(
            PathConstraints constraints,
            double kSpeedAt12VoltsMps,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.Table = NetworkTableInstance.getDefault().getTable("Subsystems").getSubTable(simpleName);

        zeroGyro();

        // Autos
        this.ppConstraints = constraints;
        configurePathPlanner();

        cachedFieldCentricFacing.HeadingController = new PhoenixPIDController(8, 0, 0);

        // Vistion
        setVisionMeasurementStdDevs(new Matrix<N3, N1>(new SimpleMatrix(new double[] { 1.0, 1.0, 1.0 })));
        // Logging
        ntPose2d = Table.getStructTopic("Pose2d", new Pose2dStruct()).publish();
        ntPose3d = Table.getStructTopic("Pose3d", new Pose3dStruct()).publish();
        ntSwerveModuleState = Table.getStructArrayTopic("SwerveModuleState", SwerveModuleState.struct).publish();
        ntSwerveModuleTarget = Table.getStructArrayTopic("SwerveModuleTarget", SwerveModuleState.struct).publish();
        visionPose2d = Table.getStructTopic("RobotPose", new Pose2dStruct()).publish();

        registerTelemetry((s) -> {
            ntPose2d.set(s.Pose);
            state.set(s);
            ntPose3d.set(new Pose3d(s.Pose));
            ntSwerveModuleState.set(s.ModuleStates);
            ntSwerveModuleTarget.set(s.ModuleTargets);
        });

        if (Utils.isSimulation()) {
            startSimThread();
        }

        register();
    }

    // ---------- Generic Control ----------

    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d pose() {
        return state.get().Pose;
    }

    public boolean hasPose() {
        return hasPose;
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /** Returns whether the drivetrain is facing the stored rotational override. */
    public boolean isAimed() {
        if (rotationOverride == null) {
            return false;
        }
        double rotationError = Math.abs(pose().getRotation().minus(rotationOverride).getRadians());
        // TODO: check logic worried about bounds being [-pi,pi]
        return rotationError <= autoAimDeadband.getRadians();
    }

    /**
     * Sets a rotational override. Provide a value of `null` to clear the override.
     */
    public void setRotationOverride(Rotation2d rotation) {
        rotationOverride = rotation;
    }

    private Optional<Rotation2d> getRotationTargetOverride() {
        return Optional.ofNullable(rotationOverride);
    }

    // ---------- PathPlanner/Auto ----------
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                this::pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        SwerveConfig.kSpeedAt12VoltsMps,
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

    public void addPPAutos(SendableChooser<Command> autos) {
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autos.addOption(autoName + " PP", AutoBuilder.buildAuto(autoName));
        }
    }

    // ---------- Commands ----------

    public Command zeroGyro() {
        return runOnce(() -> {
            // hasPose = false;
            seedFieldRelative(new Pose2d());
            // new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218),
            // Rotation2d.fromDegrees(0)));
        });

        // return runOnce(() -> {
        // hasPose = false;
        // seedFieldRelative(
        // new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218),
        // Rotation2d.fromDegrees(270)));
        // });
    }

    public Command zeroGyroToSubwoofer() {
        return runOnce(() -> {
            // hasPose = false;
            seedFieldRelative(
                    new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218),
                            Rotation2d.fromDegrees(0)));
        });
    }

    public Command driveTo(Pose2d target, double velocity) {
        return AutoBuilder.pathfindToPose(target, ppConstraints, velocity);
    }

    public Command DriveToThenPath(PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(path, ppConstraints, 0);
    }

    /**
     * Values are squared!!!! this adds more perdition at low speeds
     *
     * @param xDutyCycle [-1,1] percent speed + is forward
     * @param yDutyCycle [-1,1] percent speed + is Left
     * @param rDutyCycle [-1, 1] percent angler rate + CC+ // TODO: check this value
     * @return
     */

    public Command driveDutyCycle(DoubleSupplier xDutyCycle, DoubleSupplier yDutyCycle, DoubleSupplier rDutyCycle) {
        return applyRequest(() -> {
            double x = xDutyCycle.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
            double y = yDutyCycle.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
            double r = rDutyCycle.getAsDouble() * SwerveConfig.kMaxAngularRate;

            if (rotationOverride == null) {
                return cachedFieldCentric.withVelocityX(x).withVelocityY(y)
                        .withRotationalRate(r);
            } else {
                System.out.println("fasing angle");

                return cachedFieldCentricFacing.withVelocityX(x).withVelocityY(y)
                        .withTargetDirection(rotationOverride);
            }

        });
    }

    public Command driveVelocity(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rVel) {
        return applyRequest(() -> {
            double x = xVel.getAsDouble();
            double y = yVel.getAsDouble();
            double r = rVel.getAsDouble();
            return cachedFieldCentric.withVelocityX(x).withVelocityY(y)
                    .withRotationalRate(r);
        });
    }

    public Command brake() {
        return applyRequest(
                () -> cachedBrake);
    }

    // ---------- Vision ----------

    // Forward Camera
    private PhotonCamera cam = new PhotonCamera("cam1");
    private Transform3d robotToCam = new Transform3d(Units.inchesToMeters(13.5 - 0.744844), 0,
            Units.inchesToMeters(7.5 + 1.993), new Rotation3d(0, Units.degreesToRadians(-37.5), 0));

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Construct PhotonPoseEstimator

    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void addPhotonVision() {
        Optional<EstimatedRobotPose> robotPose = getEstimatedGlobalPose(pose());
        if (robotPose.isPresent()) {
            addVisionMeasurement(robotPose.get().estimatedPose.toPose2d(),
                    robotPose.get().timestampSeconds);
            visionPose2d.set(robotPose.get().estimatedPose.toPose2d());
        }
    }

    @Override
    public void periodic() {
        addPhotonVision();
    }

    // ---------- Sim ----------
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

    // ---------- Logging ----------
    @Override
    public void log() {
        // TODO: add motor data

    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}
