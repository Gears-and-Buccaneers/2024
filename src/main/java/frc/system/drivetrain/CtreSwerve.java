package frc.system.drivetrain;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Drivetrain;
import frc.system.Vision.Measurement;

public class CtreSwerve extends SwerveDrivetrain implements Drivetrain {
    private final PathConstraints constraints;

    private final SwerveRequest.SwerveDriveBrake cachedBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric cachedFieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.FieldCentricFacingAngle cachedFieldCentricFacing = new SwerveRequest.FieldCentricFacingAngle();

    private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();

    public CtreSwerve(PathConstraints constraints, double kSpeedAt12VoltsMps,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        registerTelemetry((s) -> state.set(s));

        SwerveRequest.ApplyChassisSpeeds req = new SwerveRequest.ApplyChassisSpeeds();

        double driveBaseRadius = 0;

        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                this::pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                () -> m_kinematics.toChassisSpeeds(state.get().ModuleStates),
                (speeds) -> setControl(req.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().filter(a -> a == Alliance.Red).isPresent(), // Change this if the path
                                                                                              // needs to be
                // flipped on red vs blue
                this);

        this.constraints = constraints;
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

    @Override
    public Pose2d pose() {
        return state.get().Pose;
    }

    // Commands
    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public Command driveTo(Pose2d target, double velocity) {
        return AutoBuilder.pathfindToPose(target, constraints, velocity);
    }

    @Override
    public Command drive(double xVel, double yVel, double rVel) {
        return applyRequest(
                () -> cachedFieldCentric.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rVel));
    }

    @Override
    public Command driveFacing(double xVel, double yVel, Rotation2d target) {
        return applyRequest(
                () -> (cachedFieldCentricFacing.withVelocityX(xVel).withVelocityY(yVel).withTargetDirection(target)));
    }

    @Override
    public Command brake() {
        return applyRequest(
                () -> cachedBrake);
    }
}
