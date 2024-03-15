package frc.system;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Pivot implements Subsystem {
    /**
     * The shooter degree offset from being perpendicular to the arm, in radians.
     */
    final double pivotBarPlatformRads;
    final double offsetToShooterPlane;

    private final double defaultDeadband;
    /** The position for intaking, in rotations. */
    private final double intakePosition;
    /** The position for shooting into the amp, in rotations. */
    private final double ampPosition;
    private final double armLength;

    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    // Network
    private NetworkTable Table;

    private DoubleSubscriber ka;
    private DoubleSubscriber kd;
    private DoubleSubscriber kg;
    private DoubleSubscriber ki;
    private DoubleSubscriber kp;
    private DoubleSubscriber ks;
    private DoubleSubscriber kv;

    private DoubleSubscriber speed;
    private DoubleSubscriber speed3;

    private final Supplier<Translation3d> vectorToSpeaker;

    private final double subwooferAngleRads = -0.0530455469;

    /**
     * Assumes 90 degree angle is 0 and the intake position has negative rotational
     * sign.
     */
    public Pivot(NetworkTable networkTable, Supplier<Translation3d> vectorToSpeaker) {
        // TODO: the pivot
        this.defaultDeadband = 0.01;

        this.intakePosition = -7.161 / 100;
        this.ampPosition = 0.25;
        this.armLength = 0.4318;
        this.vectorToSpeaker = vectorToSpeaker;

        // 11 in

        pivotBarPlatformRads = Units.degreesToRadians(52);
        offsetToShooterPlane = armLength * Math.sin(pivotBarPlatformRads);

        // Motors
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new TalonFX(12);
        rightMotor = new TalonFX(13);

        configPID();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(toIntake());
    }

    public void configPID() {
        ka = Table.getDoubleTopic("a").subscribe(0);
        this.Table.getDoubleTopic("a").publish();
        kd = Table.getDoubleTopic("d").subscribe(0);
        this.Table.getDoubleTopic("d").publish();
        kg = Table.getDoubleTopic("g").subscribe(0);
        this.Table.getDoubleTopic("g").publish();
        ki = Table.getDoubleTopic("i").subscribe(0);
        this.Table.getDoubleTopic("i").publish();
        kp = Table.getDoubleTopic("p").subscribe(0);
        this.Table.getDoubleTopic("p").publish();
        ks = Table.getDoubleTopic("s").subscribe(0);
        this.Table.getDoubleTopic("s").publish();
        kv = Table.getDoubleTopic("v").subscribe(0);
        this.Table.getDoubleTopic("v").publish();

        this.Table.getDoubleTopic("speed").publish();
        speed = Table.getDoubleTopic("speed").subscribe(0.3);
        this.Table.getDoubleTopic("speed2").publish();
        speed3 = Table.getDoubleTopic("speed2").subscribe(0.3);

        TalonFXConfiguration pivotConf = new TalonFXConfiguration();
        pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConf.Feedback.SensorToMechanismRatio = 100;
        pivotConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConf.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConf.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;

        pivotConf.Slot0.kP = 13;
        pivotConf.Slot0.kG = 0.27; // TODO: check this val
        pivotConf.Slot0.kV = 1.88;
        pivotConf.Slot0.kA = 0.01;

        // pivotConf.CurrentLimits.StatorCurrentLimit = 75;
        // pivotConf.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConf.CurrentLimits.SupplyCurrentLimit = 75; // TODO:pick a number
        pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
        // pivotConf.CurrentLimits.SupplyTimeThreshold = 0;

        // pivotConf.Slot0.kD = kd.getAsDouble();
        // pivotConf.Slot0.kI = ki.getAsDouble();
        // pivotConf.Slot0.kP = kp.getAsDouble();
        // pivotConf.Slot0.kS = 70.0;

        pivotConf.MotionMagic.MotionMagicAcceleration = 0.5;
        pivotConf.MotionMagic.MotionMagicCruiseVelocity = 1;

        pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(pivotConf);
        pivotConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(pivotConf);

        atIntake();

        // startMech();
    }

    private double rotationsToSpeaker() {
        Translation3d vector = vectorToSpeaker.get();
        double distance = vector.getNorm();
        double pitch = Math.asin(vector.getZ() / distance);
        return Units.radiansToRotations(
                Math.PI - pivotBarPlatformRads - Math.asin(offsetToShooterPlane / distance) + pitch);
    }

    public void goTo(double rotations) {
        leftMotor.setControl(new MotionMagicVoltage(rotations));
        rightMotor.setControl(new MotionMagicVoltage(rotations));
        // arm.setAngle(rotations);
    }

    public Command manual(DoubleSupplier speed1) {
        Command cmd = new Command() {
            public void execute() {
                double s = -speed1.getAsDouble() * speed.getAsDouble();

                leftMotor.setControl(new DutyCycleOut(s));
                rightMotor.setControl(new DutyCycleOut(s));
            }

            @Override
            public void end(boolean interrupted) {
                leftMotor.setControl(new DutyCycleOut(0));
                rightMotor.setControl(new DutyCycleOut(0));
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    public Command toSpeaker() {
        return goToRadCmd(subwooferAngleRads);// rotationsToSpeaker()); //TODO: add auto aming
    }

    public Command toAmp() {
        return goToRadCmd(ampPosition);
    }

    public Command toIntake() {
        return goToRadCmd(intakePosition);
    }

    public Command goToRadCmd(double rotations) {
        Command cmd = new Command() {
            @Override
            public void initialize() {
                goTo(rotations);
            }
        };
        cmd.addRequirements(this);
        return cmd;
    }

    public boolean isAimedSpeaker() {
        return isAimedTo(rotationsToSpeaker());
    }

    public boolean isAimedAmp() {
        return isAimedTo(ampPosition);
    }

    private boolean isAimedTo(double rotations) {
        return Math.abs(leftMotor.getPosition().getValueAsDouble() - rotations) < defaultDeadband;
    }

    public void atIntake() {
        leftMotor.setPosition(intakePosition);
        rightMotor.setPosition(intakePosition);
    }

    public Mechanism2d mec = new Mechanism2d(33, 4 * 12, new Color8Bit(0, 10, 100));

    public MechanismLigament2d arm;

    private void startMech() {
        arm = new MechanismLigament2d("Arm", Units.metersToInches(armLength),
                0, 8,
                new Color8Bit(100, 10, 200));
        mec.getRoot("MechThing", 4, 17).append(arm);
        SmartDashboard.putData("MechThingggy", mec);
    }
}
