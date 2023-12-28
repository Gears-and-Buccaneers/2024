package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;

public class MotorGroup implements Motor {
    private Motor[] motors;

    private String logName;

    public MotorGroup(Motor... motors) {
        this.motors = motors;
    }

    @Override
    public synchronized void runPercentOut(double num) {
        for (Motor motor : motors) {
            motor.runPercentOut(num);
        }
    }

    @Override
    public synchronized void brakeMode(boolean enable) {
        for (Motor motor : motors) {
            motor.brakeMode(enable);
        }
    }

    @Override
    public void setInverted(boolean enable) {
        DriverStation.reportWarning("This does nothing", true);
    }

    public synchronized void disable() {
        for (Motor motor : motors) {
            motor.disable();
        }
    }

    // ----------------------------------------------------------
    @Override
    public int getCanID() {
        return -1;
    }

    @Override
    public void toLog(LogTable table) {
        for (Motor motor : motors) {
            Logger.processInputs(logName + "/Motors" + motor.getCanID(), motor);
        }
    }

    @Override
    public boolean connected() {
        return true; // TODO probaly Should FIx
    }

    // Unit Testing
    @Override
    public synchronized void close() throws Exception {
        for (Motor motor : motors) {
            motor.close();
        }
    }
}
