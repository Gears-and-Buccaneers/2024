package frc.hardware;

public interface LoggedHardware extends AutoCloseable{
    // This is in here so we can add more later if we want to
    void log();

    String getName();
}
