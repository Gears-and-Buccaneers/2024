package frc.robot.Subsytems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Intake implements AutoCloseable {
    private final PWMSparkMax m_motor;
    private final DoubleSolenoid m_piston;

    public Intake() {
        m_motor = new PWMSparkMax(1);
        m_piston = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                2,
                3);
    }

    public void deploy() {
        m_piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        m_piston.set(DoubleSolenoid.Value.kReverse);
        m_motor.set(0); // turn off the motor
    }

    public void activate(double speed) {
        if (isDeployed()) {
            m_motor.set(speed);
        } else { // if piston isn't open, do nothing
            m_motor.set(0);
        }
    }

    public boolean isDeployed() {
        return m_piston.get() == DoubleSolenoid.Value.kForward;
    }

    @Override
    public void close() throws Exception {
        m_piston.close();
        m_motor.close();
    }
}