package frc.system;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Pivot implements MechanismReq {
    /**
     * The shooter degree offset from being perpendicular to the arm, in radians.
     */
    final double o;
    final double a;

    final Command gotoIntake;
    final Command gotoAmp;

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

    /**
     * Assumes 90 degree angle is 0 and the intake position has negative rotational
     * sign.
     */
    public Pivot(NetworkTable networkTable, Supplier<Translation3d> vectorToSpeaker) {
        // TODO: the pivot
        this.defaultDeadband = 0.01;

        this.intakePosition = Units.degreesToRotations(-32);
        this.ampPosition = Units.degreesToRotations(90);
        this.armLength = 0.42;
        this.vectorToSpeaker = vectorToSpeaker;

        o = Units.degreesToRadians(53);
        a = armLength * Math.sin(o);
        gotoIntake = goTo(intakePosition);
        gotoAmp = goTo(ampPosition);

        // Motors
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new TalonFX(12);
        rightMotor = new TalonFX(13);

        // Configs
        TalonFXConfiguration pivotConf = new TalonFXConfiguration();
        pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check this
        pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConf.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotConf.Feedback.SensorToMechanismRatio = 100;

        pivotConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConf.Slot0.kA = 0;
        pivotConf.Slot0.kD = 0;
        pivotConf.Slot0.kG = 0;
        pivotConf.Slot0.kI = 0;
        pivotConf.Slot0.kP = 0;
        pivotConf.Slot0.kS = 0;
        pivotConf.Slot0.kV = 0;

        pivotConf.MotionMagic.MotionMagicAcceleration = 0.2;
        pivotConf.MotionMagic.MotionMagicCruiseVelocity = 0.2;
        // conf.MotionMagic.MotionMagicJerk

        leftMotor.getConfigurator().apply(pivotConf);
        rightMotor.getConfigurator().apply(pivotConf);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        configPID();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(toIntake());
    }

    private void configPID() {
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
        pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check this
        pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConf.Feedback.SensorToMechanismRatio = 100;
        pivotConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConf.Slot0.kA = ka.getAsDouble();
        pivotConf.Slot0.kD = kd.getAsDouble();
        pivotConf.Slot0.kG = kg.getAsDouble();
        pivotConf.Slot0.kI = ki.getAsDouble();
        pivotConf.Slot0.kP = kp.getAsDouble();
        pivotConf.Slot0.kS = ks.getAsDouble();
        pivotConf.Slot0.kV = kv.getAsDouble();

        leftMotor.getConfigurator().apply(pivotConf);
        rightMotor.getConfigurator().apply(pivotConf);
    }

    public Command toSpeaker() {
        Translation3d vector = vectorToSpeaker.get();
        double distance = vector.getNorm();
        double pitch = Math.asin(vector.getZ() / distance);
        double rotations = Units.radiansToRotations(Math.PI - o - Math.asin(a / distance) + pitch);
        return goTo(rotations);
    }

    public Command manual2(DoubleSupplier speed1) {
        return new Command() {
            public void execute() {
                SmartDashboard.putNumber("SetVal", Math.round(speed1.getAsDouble()) * speed3.getAsDouble());
                leftMotor.setControl(
                        new TorqueCurrentFOC(Math.round(speed1.getAsDouble()) * speed3.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                leftMotor.setControl(new DutyCycleOut(0));
            }
        };
    }

    public Command manual(DoubleSupplier speed1) {
        return new Command() {
            public void execute() {
                leftMotor.setControl(new TorqueCurrentFOC(speed1.getAsDouble() * speed.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                leftMotor.setControl(new DutyCycleOut(0));
            }
        };
    }

    public Command congigPID() {
        return new InstantCommand(this::configPID);
    }

    public Command toAmp() {
        return gotoAmp;
    }

    public Command toIntake() {
        return gotoIntake;
    }

    public Command goTo(double rotations) {
        return new Command() {
            @Override
            public void initialize() {
                leftMotor.setControl(new MotionMagicDutyCycle(rotations));
            }

            @Override
            public boolean isFinished() {
                return Math.abs(leftMotor.getPosition().getValueAsDouble() - rotations) < defaultDeadband;
            }
        };
    }

    public void atIntake() {
        leftMotor.setPosition(intakePosition);
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disable'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
