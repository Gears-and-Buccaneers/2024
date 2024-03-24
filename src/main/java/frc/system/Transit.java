package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hardware.LoggedTalonSRX;

public class Transit implements Subsystem {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final LoggedTalonSRX transitMotor;
    private final LaserCan laserCan;

    // Network
    private NetworkTable Table;

    private final DoubleTopic transitSpeedTopic;

    // vars
    /**
     * @param distanceThreshold The LaserCAN distance threshold, in millimeters,
     *                          after which a game piece is considered to be in the
     *                          transit.
     */
    private final double threshold = 250;

    private double defaultTransitSpeed = .3; // TODO: mess around with this value
    private final DoubleSubscriber transitSpeed;

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

        // Sensors
        laserCan = new LaserCan(0);
        // laserCan.setRangingMode(); //TODO: properly config the Laser can
        // laserCan.setRegionOfInterest();
        // laserCan.setTimingBudget(null);

        // Vars
        transitSpeedTopic = Table.getDoubleTopic("transitSpeed"); // TODO: check that this works. i feal its a little
                                                                  // cleaner
        transitSpeed = transitSpeedTopic.subscribe(defaultTransitSpeed);
        transitSpeedTopic.publish();
        // OLD
        /**
         * x =
         * Table.getDoubleTopic("x").subscribe(defaultX);
         * this.Table.getDoubleTopic("x").publish();
         */

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + transitMotor.getClass().getSimpleName() + " ID:" + transitMotor.getDeviceID());
        System.out.println("\t" + laserCan.getClass().getSimpleName());

        this.log();
    }

    // ---------- Generic Control ----------

    /**
     * runs the transit with PercentOutput control w/ "transitSpeed" from NT or
     * defaultTransitSpeed if no val in NT
     * 
     * @param forwards        runs the transit forward when true and backwards when
     *                        false.
     * @param percentMaxSpeed the percent of max speed the transit should be ran at
     */
    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? transitSpeed.get(defaultTransitSpeed)
                        : -transitSpeed.get(defaultTransitSpeed);
                speed *= percentMaxSpeed;

                transitMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    /**
     * disables the transit
     */
    private void disable() {
        transitMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    /**
     * 
     * @return true if the transit has a note and false if the transit does not have
     *         a note
     */
    private boolean hasNote() {
        Measurement measurement = laserCan.getMeasurement();
        // NOTE: expanded logic so easer to understand
        if (measurement == null) {
            return false;
        }
        return measurement.distance_mm < threshold;
    }

    // ---------- Commands ----------

    /**
     * Feeds a note into the transit until it hits the sensor, then backfeeds at
     * 20%(scaled from max power) power until it is no longer detected.
     * 
     * @apiNote DEPENDS on sensors
     */
    public Command feedIn() {
        return feed(true, 1).until(() -> hasNote())
                .andThen(feed(false, 0.2).until(() -> !hasNote()));
        // TODO: check if 20% power backfeeed is to much
    }

    /**
     * Feeds the transit until a note is no longer detected.
     * 
     * @apiNote DEPENDS on sensors
     */
    public Command feedOut() {
        return feed(false, 1).until(() -> !hasNote());
    }

    public Command runForwards() {
        return feed(true, 1);
    }

    public Command runBackward() {
        return feed(false, 1);
    }

    public final Trigger hasNoteTrigger = new Trigger(this::hasNote);

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
