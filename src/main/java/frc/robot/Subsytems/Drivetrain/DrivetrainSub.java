package frc.robot.Subsytems.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable{
    public final DrivetrainRequirments drivetrainIO;

    private final String simpleName = this.getClass().getSimpleName();

    public DrivetrainSub(DrivetrainRequirments drivetrain) {
        this.drivetrainIO = drivetrain;

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + drivetrain.getClass().getSimpleName());

        drivetrain.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        Logger.processInputs(simpleName, drivetrainIO);

        drivetrainIO.loadPreferences();
    }

    @Override
    public void simulationPeriodic() {

    }
    // Commands ---------------------------------------------------------

    // ---------------------------------------------------------
    @Override
    public void close() throws Exception {
        drivetrainIO.close();
    }
}
