package frc.robot.Subsytems.Arm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;

import frc.lib.hardware.motorController.*;

public class ArmHardware implements ArmRequirments {
    private final List<Motor> basePivotMotor;

    public ArmHardware() {
        basePivotMotor = new ArrayList<Motor>();

        basePivotMotor.set(0, new Falcon500(1));
        basePivotMotor.set(1, new Falcon500(2));
        basePivotMotor.set(2, new Falcon500(3));
        basePivotMotor.set(3, new Falcon500(4));

        basePivotMotor.forEach((motor) -> {
            configMotor(motor);
        });

        basePivotMotor.get(0).inverted(false);
        basePivotMotor.get(1).inverted(false);
        basePivotMotor.get(2).inverted(true);
        basePivotMotor.get(3).inverted(true);
    }

    @Override
    public void loadPreferences() {

    }

    public void setBrakeMode(boolean enable) {
        basePivotMotor.forEach((motor) -> {
            motor.brakeMode(enable);
        });
    }

    @Override
    public void toLog(LogTable table) {
        basePivotMotor.forEach((motor) -> {
            table.put("Motor" + motor.getCanID(), motor);
        });
    }

    @Override
    public void close() throws Exception {
        basePivotMotor.get(0).close();
        basePivotMotor.get(1).close();
        basePivotMotor.get(2).close();
        basePivotMotor.get(3).close();
    }

    private void configMotor(Motor motor) {
        motor.inverted(true);
    }
}
