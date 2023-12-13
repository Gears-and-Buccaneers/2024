package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase implements AutoCloseable {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSub(IntakeIO intakeIO) {
        System.out.println("[Init] Creating Intake w/ " + intakeIO.getClass().getSimpleName());
        this.intakeIO = intakeIO;

        intakeIO.setBrakeMode(false);
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public IntakeIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public Command intakePice() {
        return run(() -> {
            intakeIO.extend();
        }).andThen(run(() -> {
            if (inputs.isDeployed) {
                intakeIO.setVoltage(6);
            } else {
                DriverStation.reportWarning("Can not spin motoer. Intake is not deploed", false);
            }
        }));
    }

    public Command ejectPice() {
        return run(() -> {
            intakeIO.extend();
        }).andThen(run(() -> {
            if (inputs.isDeployed) {
                intakeIO.setVoltage(-6);
            } else {
                DriverStation.reportWarning("Can not spin motoer. Intake is not deploed", false);
            }
        }));
    }

    public Command extend() {
        return run(() -> {
            intakeIO.extend();
        });
    }

    public Command retract() {
        return run(() -> {
            intakeIO.retract();
        });
    }

    @Override
    public void close() throws Exception {
        intakeIO.close();
    }
}
