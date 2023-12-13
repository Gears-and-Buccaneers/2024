package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class IntakeSub implements AutoCloseable {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSub(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

}
