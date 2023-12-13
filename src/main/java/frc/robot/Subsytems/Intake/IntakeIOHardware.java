package frc.robot.Subsytems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.lib.hardware.motorController.*;

public class IntakeIOHardware implements IntakeIO, AutoCloseable {
    private final AdcanvedMotor motor;
    private final DoubleSolenoid m_piston;

    public IntakeIOHardware() {
        motor = new SparkMax(10);
        m_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

        // do some config stuff

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrentAmps = new double[] { motor.getOutputCurrent() };
        inputs.motorTempCelcius = new double[] { motor.getMotorTemperature() };
        inputs.isDeployed = m_piston.get() == DoubleSolenoid.Value.kForward;
    }

    public void extend() {
        m_piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        m_piston.set(DoubleSolenoid.Value.kReverse);
        motor.setVolts(0); // turn off the motor
    }

    public void setVoltage(double speed) {
        motor.setVolts(speed / 12);
    }

    public boolean isDeployed() {
        return m_piston.get() == DoubleSolenoid.Value.kForward;
    }

    @Override
    public void close() throws Exception {
        m_piston.close();
        motor.close();
    }
}
