package frc.robot.Subsytems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase implements AutoCloseable {
    public final ArmRequirments armIO;

    private final String simpleName = this.getClass().getSimpleName();

    public ArmSub(ArmRequirments armIO) {
        this.armIO = armIO;

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + armIO.getClass().getSimpleName());

        armIO.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        Logger.processInputs(simpleName, armIO);

        armIO.loadPreferences();
        armIO.periodic();
    }

    @Override
    public void simulationPeriodic() {

    }

    // Commands ---------------------------------------------------------
    public Command IntakePosition() {
        return run(() -> {
            armIO.wristAngleSetpoint(Rotation2d.fromDegrees(270));
            armIO.elevatorAngleSetpoint(Rotation2d.fromDegrees(15));
            armIO.elevatorLengthSetpoint(50);
        }).handleInterrupt(() -> {
            armIO.disable();
        });
    }

    public Command OutakePositon() {
        return run(() -> {
            armIO.wristAngleSetpoint(Rotation2d.fromDegrees(125));
            armIO.elevatorAngleSetpoint(Rotation2d.fromDegrees(130));
            armIO.elevatorLengthSetpoint(70);
        }).handleInterrupt(() -> {
            armIO.disable();
        });
    }

    // ---------------------------------------------------------
    @Override
    public void close() throws Exception {
        armIO.close();
    }
}
