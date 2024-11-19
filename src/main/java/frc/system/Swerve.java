package frc.system;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.config.SwerveConfig;

public class Swerve extends SwerveDrivetrain implements Subsystem {
    // Control Modes
    private final SwerveDriveBrake cachedBrake = new SwerveDriveBrake();
    private final FieldCentric cachedFieldCentric = new FieldCentric();

    // Network
    private final NetworkTable Table;
    private final StructPublisher<Pose2d> ntPose2d;
    private final StructPublisher<Pose3d> ntPose3d;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleState;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleTarget;

    public Swerve(
            double kSpeedAt12VoltsMps,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.Table = NetworkTableInstance.getDefault().getTable("Subsystems").getSubTable(this.getClass().getSimpleName());

        zeroGyro();

        // Logging
        ntPose2d = Table.getStructTopic("Pose2d", new Pose2dStruct()).publish();
        ntPose3d = Table.getStructTopic("Pose3d", new Pose3dStruct()).publish();
        ntSwerveModuleState = Table.getStructArrayTopic("SwerveModuleState", SwerveModuleState.struct).publish();
        ntSwerveModuleTarget = Table.getStructArrayTopic("SwerveModuleTarget", SwerveModuleState.struct).publish();
        Field2d f = new Field2d();

        registerTelemetry((s) -> {
            ntPose2d.set(s.Pose);
            ntPose3d.set(new Pose3d(s.Pose));
            ntSwerveModuleState.set(s.ModuleStates);
            ntSwerveModuleTarget.set(s.ModuleTargets);
            f.setRobotPose(s.Pose);
            SmartDashboard.putData("field", f);
        });

        register();
    }

    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command zeroGyro() {
        return runOnce(() -> seedFieldRelative(new Pose2d()));
    }

    public Command driveDutyCycle(DoubleSupplier xDutyCycle, DoubleSupplier yDutyCycle, DoubleSupplier rDutyCycle) {
        return applyRequest(() -> {
            double x = xDutyCycle.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
            double y = yDutyCycle.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
            double r = rDutyCycle.getAsDouble() * SwerveConfig.kMaxAngularRate;

			return cachedFieldCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(r);
        });
    }

    public Command brake() {
        return applyRequest(() -> cachedBrake);
    }
}
