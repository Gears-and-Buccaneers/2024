package frc.lib.hardware.sensor.imu;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * refer to https://docs.wpilib.org/en/latest/docs/hardware/sensors/proximity-switches.html for more infomation
 */
public interface IMURequirments {
    double getPitch();
    double getYaw();
    double getRoll();


    public default Rotation3d getRotation3d() {
        return new Rotation3d(getRoll(), getPitch(), getYaw());
    }
}
