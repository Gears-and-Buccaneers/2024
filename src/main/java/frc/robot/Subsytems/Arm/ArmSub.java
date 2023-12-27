package frc.robot.Subsytems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase implements AutoCloseable {
    public final ArmRequirments armIO;

    private final String simpleName = this.getClass().getSimpleName();

    public ArmSub(ArmRequirments armIO) {
        this.armIO = armIO;

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + armIO.getClass().getSimpleName());
    }

    // ---------------------------------------------------------
    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}
