package frc.robot.Subsytems.Example;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSub extends SubsystemBase implements AutoCloseable{
    public final ExampleRequirments ExampleIO;

    private final String simpleName = this.getClass().getSimpleName();

    public ExampleSub(ExampleRequirments ExampleIO) {
        this.ExampleIO = ExampleIO;

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + ExampleIO.getClass().getSimpleName());

        ExampleIO.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        Logger.processInputs(simpleName, ExampleIO);

        ExampleIO.loadPreferences();
    }

    @Override
    public void simulationPeriodic() {

    }
    // Commands ---------------------------------------------------------

    // ---------------------------------------------------------
    @Override
    public void close() throws Exception {
        ExampleIO.close();
    }
}
