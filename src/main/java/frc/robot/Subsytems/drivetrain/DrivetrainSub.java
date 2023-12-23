package frc.robot.Subsytems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.joystics.Oporator;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable {
    private final DrivetrainRequirments drivetrain;

    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    public DrivetrainSub(DrivetrainRequirments drivetrain) {
        this.drivetrain = drivetrain;
        System.out.println("[Init] Creating"
                + this.getClass().getSimpleName() + " w/ "
                + this.drivetrain.getClass().getSimpleName());
    }

    @Override
    public void periodic() {
        drivetrain.updateInputs(inputs);
        Logger.processInputs(this.getClass().getSimpleName(), inputs);

        if (getInputs().GyroConnected) {
            odometry.update(getInputs().yaw, getDrivetrainState());
        }

        var pose = odometry.getPoseMeters();
        field.setRobotPose(pose);

        System.out.println("pose: " + pose.toString());

        SmartDashboard.putString("Chassie Speed Calcualted", getChassisSpeed().toString());
        // SmartDashboard.putData(imu);
        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("Odometry yaw", pose.getRotation().getDegrees());

        for (ModuleBase module : modules) {
            SmartDashboard.putData("Mod" + module.number + ": State", module);
        }
    }

    public DrivetrainInputsAutoLogged getInputs() {
        return inputs;
    }

    /**
     * drives the drivetrain
     *
     * @param translation      the x and y compnet of the movment (meaters/second)
     * @param rotation         the rotation of the robot (radians/second)
     * @param notFieldRelative false for field relitive
     */
    public Command drive(Oporator controler, boolean fieldRelative) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    controler.getDrivtrainTranslationX(),
                    controler.getDrivtrainTranslationY(),
                    controler.getDrivtrainRotation());
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getInputs().yaw);
            drivetrain.setChassisSpeed(speeds);
        });
    }

    @Override
    public void close() throws Exception {
        drivetrain.close();
    }
}