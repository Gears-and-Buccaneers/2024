package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.hardware.*;
import edu.wpi.first.networktables.*;

public class Transit implements LoggedSubsystems {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final LoggedTalonSRX transitMotor;
    private final LoggedLaserCan laserCan;

    // Network
    private final NetworkTable Table;

    private final DoubleTopic transitSpeedTopic;

    // Vars
    /**
     * @param distanceThreshold The LaserCAN distance threshold, in millimeters,
     *                          after which a game piece is considered to be in the
     *                          transit.
     */
    private final double threshold = 250;

    private final double defaultTransitSpeed = .3; // TODO: mess around with this value
    private final DoubleSubscriber transitSpeed;

    // Logging
    private final BooleanPublisher hasNote;

    public Transit(NetworkTable networkTable) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        transitMotor = new LoggedTalonSRX(11, this.Table, "transitMotor");

        // Configs
        transitMotor.setInverted(true);

        transitMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();
        currentLimits.currentLimit = 40;// TODO: find/check value for current limit
        currentLimits.enable = true;

        transitMotor.configSupplyCurrentLimit(currentLimits);

        // Sensors
        laserCan = new LoggedLaserCan(0, Table, "LaserCan");
        // laserCan.setRangingMode(); //TODO: properly config the Laser can
        // laserCan.setRegionOfInterest();
        // laserCan.setTimingBudget(null);

        // Vars
        transitSpeedTopic = Table.getDoubleTopic("transitSpeed"); // TODO: check that this works. i feal its better
        transitSpeed = transitSpeedTopic.subscribe(defaultTransitSpeed);
        transitSpeedTopic.publish();
        // OLD
        /**
         * x =
         * Table.getDoubleTopic("x").subscribe(defaultX);
         * this.Table.getDoubleTopic("x").publish();
         */

        // log
        hasNote = Table.getBooleanTopic("HasNote").publish();

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + transitMotor.getClass().getSimpleName() + " ID:" + transitMotor.getDeviceID());
        System.out.println("\t" + laserCan.getClass().getSimpleName());

        this.log();
        register();
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
    public boolean hasNote() {
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
                .andThen(feed(false, .8).until(() -> !hasNote()))
                .andThen(feed(true, 0.8).until(() -> hasNote()));
        // TODO: check if 20% power backfeeed is to much
    }

    /**
     * Feeds the transit until a note is no longer detected.
     * 
     * @apiNote DEPENDS on sensors
     */
    public Command feedOut() {
        return feed(true, 1).until(() -> hasNote()).until(() -> !hasNote());
    }

    public Command runForwards() {
        return feed(true, 1);
    }

    public Command runBackward() {
        return feed(false, 1);
    }

    public final Trigger hasNoteTrigger = new Trigger(this::hasNote);

    // ---------- Logging ----------
    @Override
    public void log() {
        // Subsystem states
        hasNote.set(hasNote());

        // Hardware
        transitMotor.log();
        // laserCan.log(); //I think this should be called w/ has note
    }

    @Override
    public void close() {
        transitMotor.close();

        hasNote.close();
    }
}
