package frc.robot.Subsytems.Arm;

import org.littletonrobotics.junction.LogTable;

import frc.lib.hardware.motorController.*;

public class ArmHardware implements ArmRequirments {
    private final Motor motor1;
    private final Motor motor2;
    private final Motor motor3;
    private final Motor motor4;

    public ArmHardware() {
        motor1 = new Falcon500(1);
        motor2 = new Falcon500(2);
        motor3 = new Falcon500(3);
        motor4 = new Falcon500(4);

        configMotor(motor1);
        configMotor(motor2);
        configMotor(motor3);
        configMotor(motor4);
    }

    @Override
    public void loadPreferences() {

    }

    public void setBrakeMode(boolean enable) {
        motor1.brakeMode(enable);
        motor2.brakeMode(enable);
        motor3.brakeMode(enable);
        motor4.brakeMode(enable);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Motor1", motor1);
        table.put("Motor2", motor2);
        table.put("Motor3", motor3);
        table.put("Motor4", motor4);
    }

    @Override
    public void close() throws Exception {
        motor1.close();
        motor2.close();
        motor3.close();
        motor4.close();
    }

    private void configMotor(Motor motor) {
        motor.inverted(true);
    }
}
