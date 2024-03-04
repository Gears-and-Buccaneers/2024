package frc.system.Pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.MechanismReq;

public class Pivot implements MechanismReq {
    final double o;
    final double a;

    final Command gotoIntake;
    final Command gotoAmp;

    private final double defaultDeadband;
    private final Rotation2d intakePosition;
    private final Rotation2d ampPosition;
    private final double armLength;
    private final Rotation2d armOffset;

    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    // Network
    private NetworkTable Table;

    /**
     * Assumes 90 degree angle is 0 and the intake position has negative rotational
     * sign.
     */
    public Pivot(NetworkTable networkTable) {
        // TODO: the pivot
        this.defaultDeadband = 0.01;
        this.intakePosition = Rotation2d.fromDegrees(32);
        this.ampPosition = Rotation2d.fromDegrees(130);
        this.armLength = .42;
        this.armOffset = Rotation2d.fromDegrees(53);

        o = armOffset.getRadians();
        a = armLength * Math.sin(armOffset.getRadians());
        gotoIntake = goTo(intakePosition.getRotations());
        gotoAmp = goTo(ampPosition.getRotations());

        // Motors
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new TalonFX(12);
        rightMotor = new TalonFX(13);

        // Configs
        TalonFXConfiguration pivotConf = new TalonFXConfiguration();
        pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check this
        pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    }

    public Command toSpeaker(double pitchRad, double dist) {
        double rotations = Units.radiansToRotations(Math.PI - o - Math.asin(a / dist) + pitchRad);
        return goTo(rotations);
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
