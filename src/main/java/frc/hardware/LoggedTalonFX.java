package frc.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

// Network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LoggedTalonFX extends TalonFX implements LoggedHardware {

    private final NetworkTable Table;
    private String name;

    // Logged Data
    private final StringPublisher ControlMode;

    public LoggedTalonFX(int deviceNumber, NetworkTable networkTable, String name) {
        super(deviceNumber);

        this.name = name + "" + this.getDeviceID();
        this.Table = networkTable.getSubTable(name);

        // Logged Data
        ControlMode = Table.getStringTopic("ControlMode").publish();
    }

    @Deprecated
    public LoggedTalonFX(int deviceNumber, NetworkTable networkTable) {
        this(deviceNumber, networkTable, "Motor");
    }

    @Deprecated
    public LoggedTalonFX(int deviceNumber) {
        this(deviceNumber, NetworkTableInstance.getDefault().getTable("Hardware"));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void log() {
        ControlMode.set(this.getControlMode().toString());

        // Table.getStringArrayTopic("ControlMode").publish()
        // .set(new String[] {
        // leftMotor.getControlMode().toString(),
        // rightMotor.getControlMode().toString() });
        // Table.getIntegerArrayTopic("DeviceID").publish()
        // .set(new long[] {
        // leftMotor.getDeviceID(),
        // rightMotor.getDeviceID() });

        // Table.getDoubleArrayTopic("set speed").publish()
        // .set(new double[] {
        // leftMotor.get(),
        // rightMotor.get() });

        // Table.getDoubleArrayTopic("getVelocity").publish()
        // .set(new double[] {
        // leftMotor.getVelocity().getValueAsDouble(),
        // rightMotor.getVelocity().getValueAsDouble() });

        // Table.getDoubleArrayTopic("getDeviceTemp").publish()
        // .set(new double[] {
        // leftMotor.getDeviceTemp().getValueAsDouble(),
        // rightMotor.getDeviceTemp().getValueAsDouble() });

        // Table.getDoubleArrayTopic("getMotorVoltage").publish()
        // .set(new double[] {
        // leftMotor.getMotorVoltage().getValueAsDouble(),
        // rightMotor.getMotorVoltage().getValueAsDouble() });

        // // Table.getDoubleArrayTopic("Temp").publish()
        // // .set(new double[] { leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp()
        // });
        // // Table.getDoubleArrayTopic("Supply Current").publish()
        // // .set(new double[] { leftMotor.getSupplyCurrent(),
        // // rightMotor.getSupplyCurrent() });
        // // Table.getDoubleArrayTopic("Stator Current").publish()
        // // .set(new double[] { leftMotor.getStatorCurrent(),
        // // rightMotor.getStatorCurrent() });
        // // Table.getDoubleArrayTopic("Motor Voltage").publish()
        // // .set(new double[] { leftMotor.getMotorVoltage(),
        // rightMotor.getMotorVoltage()
        // // });
    }

    public void close() {
        super.close();
        ControlMode.close();
    }

}
