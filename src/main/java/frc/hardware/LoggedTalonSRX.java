package frc.hardware;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
// Network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LoggedTalonSRX extends TalonSRX implements LoggedHardware {

    private final NetworkTable Table;
    private String name;

    // Logged Data
    private final StringPublisher ControlMode;
    private final IntegerPublisher BaseID;
    private final IntegerPublisher DeviceID;
    private final IntegerPublisher FirmwareVersion;
    private final BooleanPublisher Inverted;
    private final DoublePublisher MotorOutputPercent;
    private final DoublePublisher MotorOutputVoltage;
    private final DoublePublisher StatorCurrent;
    private final DoublePublisher Temperature;

    public LoggedTalonSRX(int deviceNumber, NetworkTable networkTable, String name) {
        super(deviceNumber);

        this.name = name + "" + this.getDeviceID();
        this.Table = networkTable.getSubTable(name);

        // Logged Data
        ControlMode = Table.getStringTopic("ControlMode").publish();
        BaseID = Table.getIntegerTopic("BaseID").publish();
        DeviceID = Table.getIntegerTopic("DeviceID").publish();
        FirmwareVersion = Table.getIntegerTopic("FirmwareVersion").publish();
        Inverted = Table.getBooleanTopic("Inverted").publish();
        MotorOutputPercent = Table.getDoubleTopic("MotorOutputPercent").publish();
        MotorOutputVoltage = Table.getDoubleTopic("MotorOutputVoltage").publish();
        StatorCurrent = Table.getDoubleTopic("StatorCurrent").publish();
        Temperature = Table.getDoubleTopic("Temperature").publish();
    }

    @Deprecated
    public LoggedTalonSRX(int deviceNumber, NetworkTable networkTable) {
        this(deviceNumber, networkTable, "Motor");
    }

    @Deprecated
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
        BaseID.set(this.getBaseID());
        DeviceID.set(this.getDeviceID());
        FirmwareVersion.set(this.getFirmwareVersion());
        Inverted.set(this.getInverted());
        MotorOutputPercent.set(this.getMotorOutputPercent());
        MotorOutputVoltage.set(this.getMotorOutputVoltage());
        StatorCurrent.set(this.getStatorCurrent());
        Temperature.set(this.getTemperature());

    }

    @Override
    public void close() {
        ControlMode.close();
    }
}
