package frc.system;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.hardware.LoggedTalonFX;

public class Shooter implements LoggedSubsystems {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final LoggedTalonFX leftMotor;
    private final LoggedTalonFX rightMotor;
    private DutyCycleOut DutyCycleCtrlMode = new DutyCycleOut(0);

    // Network
    /** the sub table where all logging for Shooter should go */
    private final NetworkTable shooterTable;

    /** the speed for shooting in Speaker */
    private final DoubleSubscriber speakerSpeed;

    private final DoublePublisher setShooterSpeed;
    // Vars
    /** the max RMP of the shooting wheals */
    private final double maxShooterSpeed = 6000;
    /** the default Speaker RMP for shooting */
    private final double defaultSpeakerSpeed = 5000;
    /** amount of accseptable error for shooter in RPM */
    private final double deadBand = 250;

    // ---------- Config ----------
    public Shooter(NetworkTable networkTable) {
        // Network tables
        this.shooterTable = networkTable.getSubTable(simpleName);

        this.shooterTable.getDoubleTopic("speakerSpeed").publish();
        speakerSpeed = shooterTable.getDoubleTopic("speakerSpeed").subscribe(defaultSpeakerSpeed);

        setShooterSpeed = this.shooterTable.getDoubleTopic("setShooterSpeed").publish();

        // Motors
        leftMotor = new LoggedTalonFX(14, this.shooterTable, "leftShooter");
        rightMotor = new LoggedTalonFX(15, this.shooterTable, "rightShooter");

        configMotor();

        // Logging
        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
        System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

        this.log();

        register();
    }

    /**
     * configs both of the talonFX
     * <ol>
     * <li>NeutralMode</li>
     * <li>gearRatio</li>
     * <li>motion magic values/PID values</li>
     * <li>curet limit</li>
     * <li>inversion</li>
     * </ol>
     */
    public void configMotor() {
        TalonFXConfiguration shooterConf = new TalonFXConfiguration();

        shooterConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConf.Feedback.SensorToMechanismRatio = 1;

        shooterConf.Slot0.kV = 0.18;
        shooterConf.Slot0.kA = 0.0;
        shooterConf.Slot0.kP = 0.1;
        shooterConf.Slot0.kI = 0.0;
        shooterConf.Slot0.kD = 0.0;

        shooterConf.CurrentLimits.SupplyCurrentLimit = 80;
        shooterConf.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftMotor.getConfigurator().apply(shooterConf);
        rightMotor.getConfigurator().apply(shooterConf);

    }

    // ---------- Generic Functions ----------

    // ---------- Command CtrlModes ----------
    /**
     * Controls the shooter based on a direction and speed.
     * Used for open loop control of the shooter
     * 
     * 
     * @param forwards  true shoots a note false feeds not into robot
     * @param dutyCycle [-{@link Shooter#maxShooterSpeed},
     *                  {@link Shooter#maxShooterSpeed}] the speed to control the
     *                  shooter at,
     *                  max speed based on maxSpeed,
     * @return a command that requires the shooter and when on ends the motors are
     *         disabled
     */
    private Command VelocityOpenLoop(boolean forwards, double RPM) {
        Command cmd = new Command() {
            public void initialize() {
                if (RPM > maxShooterSpeed || RPM < -maxShooterSpeed) {
                    DriverStation.reportWarning(
                            "setting Shooter VelocityOpenLoop RPM outside of controllable range", true);
                }
                DutyCycleCtrlMode.Output = (forwards ? RPM : -RPM) / maxShooterSpeed;

                leftMotor.setControl(DutyCycleCtrlMode);
                rightMotor.setControl(DutyCycleCtrlMode);
            }

            public void end(boolean interrupted) {
                leftMotor.disable();
                rightMotor.disable();
            }
        };

        cmd.withName(simpleName + "feed");
        cmd.addRequirements(this);
        return cmd;
    }

    /**
     * stops/disable the shooter motors
     * 
     * @return a command that requires the shooter and when on ends the motors are
     *         disabled
     */
    public Command stop() {
        Command cmd = new Command() {
            public void initialize() {
                leftMotor.disable();
                rightMotor.disable();
            }
        };

        cmd.withName(simpleName + "Stop");
        cmd.addRequirements(this);
        return cmd;
    }

    // ---------- Commands ----------
    /**
     * spins the shooter weals at a const speakerSpeed(NT)
     * 
     * @return a command that requires the Shooter and when on ends the motors are
     *         disabled
     */
    public Command shootSpeaker() {
        Command cmd = VelocityOpenLoop(true, speakerSpeed.get());
        cmd.withName(simpleName + "ShootSpeaker");

        return cmd;
    }

    /**
     * spins the shooter wheals based at speakerSpeed with fudge factors for when
     * robot is close to subwoofer.
     * 
     * @param k velocity multiplier
     * @param c added to velocity
     * @return a command that requires the piShootervot and when on ends the motors
     *         are
     *         disabled
     */
    public Command shootSpeaker(double k, double c) {
        Command cmd = VelocityOpenLoop(true, speakerSpeed.get() * k + c);
        cmd.withName(simpleName + "ShootSpeaker");

        return cmd;
    }

    /**
     * spins the shooter weals at a const speed of 1000 rpm
     * 
     * @return a command that requires the Shooter and when on ends the motors are
     *         disabled
     */
    public Command shootAmp() {
        Command cmd = VelocityOpenLoop(true, 1000);
        cmd.withName(simpleName + "ShootAmp");

        return cmd;
    }

    /**
     * spins the shooter weals at a const speed of 500 rpm BACKWARD
     * 
     * @return a command that requires the Shooter and when on ends the motors are
     *         disabled
     */
    public Command reverse() {
        Command cmd = VelocityOpenLoop(false, 500);
        cmd.withName(simpleName + "Reverse");

        return cmd;
    }

    /**
     * 
     * @return a new wait Until Command that waits until the both shooter weals are
     *         at the desired velocity
     */
    public Command waitPrimed() {// TODO: not quite sure if this will work

        double leftMotorError = Math
                .abs(leftMotor.getRotorVelocity().getValue() - DutyCycleCtrlMode.Output * maxShooterSpeed);
        double rightMotorError = Math
                .abs(rightMotor.getRotorVelocity().getValue() - DutyCycleCtrlMode.Output * maxShooterSpeed);

        return new WaitUntilCommand(() -> {
            return leftMotorError < deadBand && rightMotorError < deadBand;
        });
    }

    // ---------- Logging ----------
    public void log() {
        leftMotor.log();
        rightMotor.log();

        setShooterSpeed.set(DutyCycleCtrlMode.Output);
    }

    @Override
    public void close() throws Exception {
        // Hardware
        leftMotor.close();
        rightMotor.close();

        // Network Table
        speakerSpeed.close();

        setShooterSpeed.close();
    }
}
