package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;

    // Network
    private final NetworkTable Table;
    private DoubleSubscriber intakeSpeed;

    // vars
    private double defaultIntakeSpeed = .8;

    public Intake(NetworkTable networkTable) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new TalonSRX(9);
        rightMotor = new TalonSRX(10);

        // Configs
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();
        currentLimits.currentLimit = 40; // TODO: find/check value for current limit
        currentLimits.enable = true;

        leftMotor.configSupplyCurrentLimit(currentLimits);
        rightMotor.configSupplyCurrentLimit(currentLimits);

        // Sensors

        // Vars
        intakeSpeed = Table.getDoubleTopic("intakeSpeed").subscribe(defaultIntakeSpeed);
        this.Table.getDoubleTopic("intakeSpeed").publish();

        // init log
        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
        System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

		register();
    }
    // ---------- Generic Control ----------
    // TODO: I think all of Generic Control should be private if they are
    // controlling hardware but not sure

    /**
     * runs the intake with PercentOutput control w/ "intakeSpeed" from NT or
     * defaultIntakeSpeed if no val in NT
     *
     * @param forwards runs the intake forward when true and backwards when false.
     */
    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? intakeSpeed.get(defaultIntakeSpeed) : -intakeSpeed.get(defaultIntakeSpeed);
                speed *= percentMaxSpeed;

                leftMotor.set(TalonSRXControlMode.PercentOutput, speed);
                rightMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    /**
     * disables the intake
     */
    private void disable() {
        leftMotor.set(TalonSRXControlMode.Disabled, 0);
        rightMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    // ---------- Commands ----------

    /**
     * <pre>
     * returns a command that runs the intake FORWARD
     * when it ends it disables the intake
     * intake command requires itself
     * </pre>
     */
    public Command runIn() {
        return feed(true, 1);
    }

    /**
     * <pre>
     * returns a command that runs the intake BACKWARD
     * when it ends it disables the intake
     * intake command requires itself
     * </pre>
     */
    public Command runOut() {
        return feed(false, 1);
    }
}
