package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;

public class Falcon500 implements SmartMotor {
    private int id;

    public Falcon500(int id) {
        this.id = id;
    }

    public void close() throws Exception {

    }

    @Override
    public void runPercentOut(int num) {

    }

    @Override
    public void brakeMode(boolean enable) {
    }

    @Override
    public void inverted(boolean enable) {

    }

    @Override
    public int getCanID() {
        return id;
    }

    @Override
    public double getRotation() {
        return 10;
    }

    public void toLog(LogTable table) {
        table.put("ID", id);
    }
}
