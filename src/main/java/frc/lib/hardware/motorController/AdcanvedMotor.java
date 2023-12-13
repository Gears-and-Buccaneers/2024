package frc.lib.hardware.motorController;

public interface AdcanvedMotor {
    double getAppliedOutput();

    double getBusVoltage();

    double getOutputCurrent();

    double getMotorTemperature();

    double setVolts(double volts);

    void close();
}
