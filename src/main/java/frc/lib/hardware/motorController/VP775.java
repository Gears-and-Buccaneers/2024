package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class VP775 implements Motor {
    private final int canID;

    private TalonSRX motor;

    public VP775(int canID) {
        this.canID = canID;

        motor = new TalonSRX(canID);
    }

    @Override
    public void runPercentOut(int num) {
        motor.set(ControlMode.PercentOutput, num);
    }

    @Override
    public void brakeMode(boolean enable) {
        if (enable)
            motor.setNeutralMode(NeutralMode.Brake);
        else
            motor.setNeutralMode(NeutralMode.Coast);
    }

    /** true inverts it */
    @Override
    public void setInverted(boolean enable) {
        motor.setInverted(enable);
    }

    public void disable() {
        motor.set(ControlMode.Disabled, 0);
    }

    // ----------------------------------------------------------
    @Override
    public int getCanID() {
        return canID;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Open", 1);
    }

    @Override
    public boolean connected() {
        return true; // TODO probaly Should FIx
    }

    // Unit Testing
    @Override
    public void close() throws Exception {

    }
}
