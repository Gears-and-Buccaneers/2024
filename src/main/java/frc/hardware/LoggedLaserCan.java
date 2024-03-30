package frc.hardware;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.*;

public class LoggedLaserCan extends LaserCan implements LoggedHardware {

    private final NetworkTable Table;
    private String name;

    // Logged Data
    private final IntegerPublisher ambient;
    private final IntegerPublisher budget_ms;
    private final IntegerPublisher distance_mm;
    private final BooleanPublisher is_long;
    private final IntegerArrayPublisher roi;
    private final IntegerPublisher status;

    public LoggedLaserCan(int deviceNumber, NetworkTable networkTable, String name) {
        super(deviceNumber);

        this.name = name + "" + deviceNumber;
        this.Table = networkTable.getSubTable(name);

        // Logged Data
        ambient = Table.getIntegerTopic("ambient").publish();
        budget_ms = Table.getIntegerTopic("budget_ms").publish();
        distance_mm = Table.getIntegerTopic("distance_mm").publish();
        is_long = Table.getBooleanTopic("is_long").publish();
        roi = Table.getIntegerArrayTopic("roi").publish();
        status = Table.getIntegerTopic("status").publish();
    }

    @Override
    public String getName() {
        return name;
    }

    @Deprecated
    public LoggedLaserCan(int deviceNumber, NetworkTable networkTable) {
        this(deviceNumber, networkTable, "LaserCan");
    }

    @Deprecated
    public LoggedLaserCan(int deviceNumber) {
        this(deviceNumber, NetworkTableInstance.getDefault().getTable("Hardware"));
    }

    public void log(Measurement measurement) {
        ambient.set(measurement.ambient);
        budget_ms.set(measurement.ambient);
        distance_mm.set(measurement.ambient);
        is_long.set(measurement.is_long);
        roi.set(new long[] {
                measurement.roi.h,
                measurement.roi.w,
                measurement.roi.x,
                measurement.roi.y, });
        status.set(measurement.status);
    }

    @Override
    public void log() {
        this.log(this.getMeasurement());
    }
}
