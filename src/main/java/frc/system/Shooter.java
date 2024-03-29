package frc.system;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.hardware.LoggedTalonFX;

public class Shooter implements Subsystem {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final LoggedTalonFX leftMotor;
    private final LoggedTalonFX rightMotor;

    // Network
    private final NetworkTable Table;
    private final DoubleTopic shooterSpeedTopic;
    private final DoubleTopic shooterSpeedDeadBandTopic;

    // Vars
    /** Units: RPM */
    private final DoubleSubscriber shooterSpeed;
    private final double defaultShooterSpeed = 6000;
    /** Units: RPM error */
    private final DoubleSubscriber shooterSpeedDeadBand;
    private final double defaultShooterSpeedDeadBand = 50;

    private final double maxSpeed = 6000;

    public Shooter(NetworkTable networkTable) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new LoggedTalonFX(14, this.Table, "leftShooter");
        rightMotor = new LoggedTalonFX(15, this.Table, "rightShooter");

        // Configs
        TalonFXConfiguration shooterConf = new TalonFXConfiguration();
        shooterConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterConf.Slot0.kV = 0.18;
        shooterConf.Slot0.kA = 0.0;
        shooterConf.Slot0.kP = 0.1;
        shooterConf.Slot0.kI = 0.0;
        shooterConf.Slot0.kD = 0.0;

        // shooterConf.CurrentLimits.StatorCurrentLimit = 80;
        // shooterConf.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConf.CurrentLimits.SupplyCurrentLimit = 80;
        shooterConf.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftMotor.getConfigurator().apply(shooterConf);
        rightMotor.getConfigurator().apply(shooterConf);

        // Vars
        shooterSpeedTopic = Table.getDoubleTopic("shooterSpeed");
        shooterSpeed = shooterSpeedTopic.subscribe(defaultShooterSpeed);
        shooterSpeedTopic.publish();

        shooterSpeedDeadBandTopic = Table.getDoubleTopic("shooterSpeedDeadBand");
        shooterSpeedDeadBand = shooterSpeedDeadBandTopic.subscribe(defaultShooterSpeedDeadBand);
        shooterSpeedDeadBandTopic.publish();

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
        System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

        this.log();

        register();

        // setDefaultCommand(stop());
    }

    // ---------- Generic Control ----------
    private Command feed(boolean forwards, double speed1) {
        Command cmd = new Command() {
            public void initialize() {
                // double speed = 1;
                DutyCycleOut output = new DutyCycleOut((forwards ? speed1 : -speed1) / maxSpeed);

                leftMotor.setControl(output);
                rightMotor.setControl(output);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    public void disable() {
        leftMotor.disable();
        rightMotor.disable();
    }

    // ---------- Commands ----------
    public Command stop() {
        Command cmd = new Command() {
            public void initialize() {
                disable();
            }
        };

        cmd.addRequirements(this);
        return cmd;
    }

    public Command shootSpeaker() {
        return feed(true, shooterSpeed.getAsDouble());
    }

    public Command shootAmp() {
        return feed(true, 1000);
    }

    public Command reverse() {
        return feed(true, -500);
    }

    public Command waitPrimed() {
        // TODO: acceleration deadbanding
        // leftMotor.getAcceleration();

        boolean leftMotorAtSpeed = Math.abs(
                leftMotor.getRotorVelocity().getValue() - shooterSpeed.getAsDouble()) <= shooterSpeedDeadBand
                        .getAsDouble();
        boolean rightMotorAtSpeed = Math.abs(
                rightMotor.getRotorVelocity().getValue() - shooterSpeed.getAsDouble()) <= shooterSpeedDeadBand
                        .getAsDouble();

        return new WaitUntilCommand(() -> {
            return leftMotorAtSpeed && rightMotorAtSpeed;
        });
        // return new WaitCommand(1);
    }

    // Logging
    public void log() {

    }
}
