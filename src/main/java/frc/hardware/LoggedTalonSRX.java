package frc.hardware;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// Network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LoggedTalonSRX extends TalonSRX implements LoggedHardware {

    private final NetworkTable Table;
    private String name;

    // Logged Data
    private final StringPublisher ControlMode;

    public LoggedTalonSRX(int deviceNumber, NetworkTable networkTable, String name) {
        super(deviceNumber);

        this.name = name + "" + this.getDeviceID();
        this.Table = networkTable.getSubTable(name);

        // Logged Data
        ControlMode = Table.getStringTopic("ControlMode").publish();
    }

    public LoggedTalonSRX(int deviceNumber, NetworkTable networkTable) {
        this(deviceNumber, networkTable, "Motor");
    }

    public LoggedTalonSRX(int deviceNumber) {
        this(deviceNumber, NetworkTableInstance.getDefault().getTable("Hardware"));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void log() {
        ControlMode.set(this.getControlMode().toString());

        // Name
        // .set(this.getControlMode().toString()); // Val
        /**
         * Table.getStringArrayTopic("ControlMode").publish()
         * .set(new String[] { leftMotor.getControlMode().toString(),
         * rightMotor.getControlMode().toString() });
         * Table.getIntegerArrayTopic("DeviceID").publish()
         * .set(new long[] { leftMotor.getDeviceID(), rightMotor.getDeviceID() });
         * 
         * Table.getDoubleArrayTopic("Temp").publish()
         * .set(new double[] { leftMotor.getTemperature(), rightMotor.getTemperature()
         * });
         * Table.getDoubleArrayTopic("Supply Current").publish()
         * .set(new double[] { leftMotor.getSupplyCurrent(),
         * rightMotor.getSupplyCurrent() });
         * Table.getDoubleArrayTopic("Stator Current").publish()
         * .set(new double[] { leftMotor.getStatorCurrent(),
         * rightMotor.getStatorCurrent() });
         * Table.getDoubleArrayTopic("Output Voltage").publish()
         * .set(new double[] { leftMotor.getMotorOutputVoltage(),
         * rightMotor.getMotorOutputVoltage() });
         * Table.getDoubleArrayTopic("Bus Voltage").publish()
         * .set(new double[] { leftMotor.getBusVoltage(), rightMotor.getBusVoltage() });
         */
    }

    @Override
    public void close() throws Exception {
        ControlMode.close();
    }

}
