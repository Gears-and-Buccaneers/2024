package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;

public class Transit implements MechanismReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonSRX transitMotor;
    private final LaserCan laserCan;

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

    public Transit(NetworkTable networkTable) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        transitMotor = new TalonSRX(11);

        transitMotor.setInverted(true);

        transitMotor.setNeutralMode(NeutralMode.Coast);
        // TODO: CurrentLimit

        // LaserCan
        laserCan = new LaserCan(0);
        this.threshold = 250;

        // Vars
        transitSpeed = Table.getDoubleTopic("transitSpeed").subscribe(.4);
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

    private void runForward(boolean forwards) {
        double speed = forwards ? transitSpeed.get() : -transitSpeed.get();

        transitMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void disable() {
        transitMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    // Commands
    public Command run() {
        return new Command() {
            public void initialize() {
                runForward(true);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
    }

    public Command intake() {
        return new Command() {
            public void initialize() {
                runForward(true);
            }

            public boolean isFinished() {
                return hasNote();
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
    }

    public Command shoot() {
        return new Command() {
            public void initialize() {
                runForward(true);
            }

            public boolean isFinished() {
                return !hasNote();
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
    }

    public Command reverse() {
        return new Command() {
            public void initialize() {
                runForward(false);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
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

    public void close() throws Exception {
    }
}