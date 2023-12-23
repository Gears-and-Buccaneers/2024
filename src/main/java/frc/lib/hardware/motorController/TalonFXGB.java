package frc.lib.hardware.motorController;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DriverStation;

public class TalonFXGB implements Motor {
    private TalonFX talonFX;
    private MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

    public TalonFXGB(int canID) {
        talonFX = new TalonFX(canID);
        motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;

        talonFX.getConfigurator().apply(new TalonFXConfiguration());
        talonFX.getConfigurator().apply(motorConfigs);

    }

    public void setVoltageOut(double voltageOut) {
        talonFX.setControl(new VoltageOut(voltageOut));
    }

    public void setPercentOut(double percentOutput) {
        if (Math.abs(percentOutput) > 1) {
            DriverStation.reportError("You dumb. output outside of range", true);
        }

        talonFX.setControl(new VoltageOut(percentOutput / 12.0));
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

}
