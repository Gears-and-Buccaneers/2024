package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hardware.LoggedTalonSRX;

public class Transit implements Subsystem {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private LoggedTalonSRX transitMotor;
    private final LaserCan laserCan;
    public final Trigger hasNoteTrigger = new Trigger(this::hasNote);

    // Network
    private NetworkTable Table;
    private DoubleSubscriber transitSpeed;

    // vars
    /**
     * @param distanceThreshold The LaserCAN distance threshold, in millimeters,
     *                          after which a game piece is considered to be in the
     *                          transit.
     */
    private final double threshold;

    private double defaultTransitSpeed = .3; // TODO: mess around with this value

    public Transit(NetworkTable networkTable) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        transitMotor = new LoggedTalonSRX(11, this.Table, "transitMotor");

        transitMotor.setInverted(true);

        transitMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();
        currentLimits.currentLimit = 40;// TODO: find/check value for current limit
        currentLimits.enable = true;
        transitMotor.configSupplyCurrentLimit(currentLimits);

        // LaserCan
        laserCan = new LaserCan(0);
        this.threshold = 250;

        // Vars
        transitSpeed = Table.getDoubleTopic("transitSpeed").subscribe(.3);
        this.Table.getDoubleTopic("transitSpeed").publish();

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + transitMotor.getClass().getSimpleName() + " ID:" + transitMotor.getDeviceID());
        System.out.println("\t" + laserCan.getClass().getSimpleName());

        this.log();
    }

    public boolean hasNote() {
        Measurement measurement = laserCan.getMeasurement();
        return measurement != null && measurement.distance_mm < threshold;
    }

    // ---------- Commands ----------

    /**
     * Feeds a note into the transit until it hits the sensor, then backfeeds at 20%
     * power until it is no longer detected.
     */
    public Command feedIn() {
        return runForwards().until(this::hasNote).andThen(runAtPercent(-0.2).until(() -> !hasNote()));
        // TODO: check if 20% power backfeeed is to much
    }

    /** Feeds the transit until a note is no longer detected. */
    public Command feedOut() {
        return runForwards().until(() -> !hasNote());
    }

    /** Runs at a scaling percentage of the NetworkTables speed setpoint. */
    public Command runAtPercent(double percent) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = transitSpeed.get() * percent;
                transitMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                transitMotor.set(TalonSRXControlMode.Disabled, 0);
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    public Command runForwards() {
        return runAtPercent(1);
    }

    public Command runBackward() {
        return runAtPercent(-1);
    }

    @Deprecated
    public Command autoShoot() {
        return runForwards()
                .raceWith(new WaitUntilCommand(this::hasNote).andThen(new WaitUntilCommand(() -> !hasNote())));
    }

    // Logging
    public void log() {
        Table.getStringArrayTopic("ControlMode").publish()
                .set(new String[] { transitMotor.getControlMode().toString() });
        Table.getIntegerArrayTopic("DeviceID").publish()
                .set(new long[] { transitMotor.getDeviceID() });

        Table.getDoubleArrayTopic("Temp").publish()
                .set(new double[] { transitMotor.getTemperature() });
        Table.getDoubleArrayTopic("Supply Current").publish()
                .set(new double[] { transitMotor.getSupplyCurrent() });
        Table.getDoubleArrayTopic("Stator Current").publish()
                .set(new double[] { transitMotor.getStatorCurrent() });
        Table.getDoubleArrayTopic("Output Voltage").publish()
                .set(new double[] { transitMotor.getMotorOutputVoltage() });
        Table.getDoubleArrayTopic("Bus Voltage").publish()
                .set(new double[] { transitMotor.getBusVoltage() });
    }
}
