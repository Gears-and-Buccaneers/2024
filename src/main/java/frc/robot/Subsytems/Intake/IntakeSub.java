package frc.robot.Subsytems.Intake;

public class IntakeSub {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSub(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

}
