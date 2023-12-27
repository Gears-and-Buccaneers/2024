package frc.robot.Subsytems.Arm;

import org.littletonrobotics.junction.Logger;

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
    }

    @Override
    public void simulationPeriodic() {

    }
    // Commands ---------------------------------------------------------

    // ---------------------------------------------------------
    @Override
    public void close() throws Exception {
        armIO.close();
    }
}
