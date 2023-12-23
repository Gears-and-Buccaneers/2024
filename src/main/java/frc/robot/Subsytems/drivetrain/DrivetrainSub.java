package frc.robot.Subsytems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.joystics.Oporator;
//https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java this will help
//https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
//https://docs.wpilib.org/en/latest/docs/software/advanced-controls/filters/index.html
public class DrivetrainSub extends SubsystemBase implements AutoCloseable {
    private final DrivetrainRequirments drivetrain;

    private final DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    public Field2d field = new Field2d();

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
            drivetrain.updateOdometry(getInputs().yaw, 1.0);
        }

        field.setRobotPose(drivetrain.getPose2d());
    }

    public DrivetrainInputsAutoLogged getInputs() {
        return inputs;
    }

    public Command drive(Oporator controler, boolean fieldRelative) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    controler.getDrivtrainTranslationX(),
                    controler.getDrivtrainTranslationY(),
                    controler.getDrivtrainRotation());
            if (fieldRelative) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getInputs().yaw);
            }
            drivetrain.setChassisSpeed(speeds);
        }).handleInterrupt(() -> {
            drivetrain.stopChassis();
        });
    }

    @Override
    public void close() throws Exception {
        drivetrain.close();
    }
}