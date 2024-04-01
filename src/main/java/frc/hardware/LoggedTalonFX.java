package frc.hardware;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.networktables.DoublePublisher;
// Network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LoggedTalonFX extends TalonFX implements LoggedHardware {

    private final NetworkTable Table;
    private final String name;

    private StatusSignal<ControlModeValue> controlMode;
    private StatusSignal<Double> acceleration;
    private StatusSignal<Double> deviceTemp;
    private StatusSignal<Double> motorVoltage;
    private StatusSignal<Double> position;
    private StatusSignal<Double> torqueCurrent;
    private StatusSignal<Double> velocity;

    // private double updateFrequensy;

    // Logged Data
    private final StringPublisher ControlMode;
    private final DoublePublisher Acceleration;
    private final DoublePublisher DeviceTemp;
    private final DoublePublisher MotorVoltage;
    private final DoublePublisher Position;
    private final DoublePublisher TorqueCurrent;
    private final DoublePublisher Velocity;

    public LoggedTalonFX(int deviceNumber, NetworkTable networkTable, String name) {
        super(deviceNumber);

        this.name = name + "" + this.getDeviceID();
        this.Table = networkTable.getSubTable(name);

        controlMode = this.getControlMode();
        acceleration = this.getAcceleration();
        deviceTemp = this.getDeviceTemp();
        motorVoltage = this.getMotorVoltage();
        position = this.getPosition();
        torqueCurrent = this.getTorqueCurrent();
        velocity = this.getVelocity();

        // Logged Data
        ControlMode = Table.getStringTopic("ControlMode").publish();
        Acceleration = Table.getDoubleTopic("Acceleration").publish();
        DeviceTemp = Table.getDoubleTopic("DeviceTemp").publish();
        MotorVoltage = Table.getDoubleTopic("MotorVoltage").publish();
        Position = Table.getDoubleTopic("Position").publish();
        TorqueCurrent = Table.getDoubleTopic("ControlMode").publish();
        Velocity = Table.getDoubleTopic("Velocity").publish();
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
        ControlMode.set(this.controlMode.toString());
        Acceleration.set(this.acceleration.getValueAsDouble());
        DeviceTemp.set(this.deviceTemp.getValueAsDouble());
        MotorVoltage.set(this.motorVoltage.getValueAsDouble());
        Position.set(this.position.getValueAsDouble());
        TorqueCurrent.set(this.torqueCurrent.getValueAsDouble());
        Velocity.set(this.velocity.getValueAsDouble());
    }

    public void close() {
        super.close();
        ControlMode.close();
    }

}
