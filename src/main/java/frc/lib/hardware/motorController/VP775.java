package frc.lib.hardware.motorController;

import org.littletonrobotics.junction.LogTable;

public class VP775 extends Motor {

    public VP775(int id) {
        super(id);
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

    public void toLog(LogTable table) {
        table.put("ID", canID);
    }
}
