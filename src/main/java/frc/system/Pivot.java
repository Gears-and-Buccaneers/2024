package frc.system;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.LoggedTalonFX;

/**
 * 90 degree angle of pivot bar is 0 for setpoint
 * <p>
 * the intake position has negative rotational sign.
 * </p>
 */
public class Pivot implements LoggedSubsystems {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private final LoggedTalonFX leftMotor;
    private final LoggedTalonFX rightMotor;
    private MotionMagicDutyCycle MotionMagicCtrlMode = new MotionMagicDutyCycle(0);
    private DutyCycleOut DutyCycleCtrlMode = new DutyCycleOut(0);

    // Network
    /** the sub table where all logging for pivot should go */
    private final NetworkTable pivotTable;

    /** the max speed to manual control the robot */
    private final DoubleSubscriber dutyCycleMax;

    private final DoublePublisher setpoint;
    private final DoublePublisher actualRotation;

    // Vars
    /**
     * the center of the pivot bar relative to the center of the robot on the ground
     */
    public final Translation3d origin = new Translation3d(Units.inchesToMeters(27 / 2 - 4), 0,
            Units.inchesToMeters(21.75));
    /**
     * the length from the center of the pivot to the center of the transit Meters
     */
    public final double armLength = 0.43;
    /**
     * The shooter degree offset from being perpendicular to the arm, Radians.
     */
    public final double armOffsetRad = Units.degreesToRadians(38);
    /** i have know idea what this is */
    public final double exitDistance = armLength * Math.sin(armOffsetRad);
    /** the deadband for the pivot setpoint Rotations */
    public final double defaultDeadband = .01;
    /**
     * this is the stall current when the robot is pushing into the swerve covers
     */// TODO: tune this value
    public final double zeroStallCurrent = 60;

    /** The position for scoring in the speaker from the subwoofer, in rotations. */
    public final double subwooferPosition = -0.0530455469;
    /** The position for intaking, in rotations. */
    public final double intakePosition = -0.071;
    /** The position for shooting into the amp, in rotations. */
    public final double ampPosition = 0.25;

    /** the default max value for the dutyCycle ctrl mode */
    private final double dutyCycleMaxDefault = .3;

    // ---------- Config ----------
    public Pivot(NetworkTable networkTable) {
        // Network tables
        this.pivotTable = networkTable.getSubTable(simpleName);

        this.pivotTable.getDoubleTopic("speed").publish();
        dutyCycleMax = pivotTable.getDoubleTopic("speed").subscribe(dutyCycleMaxDefault);

        setpoint = this.pivotTable.getDoubleTopic("setpoint").publish();
        actualRotation = this.pivotTable.getDoubleTopic("actualRotation").publish();

        // Hardware
        leftMotor = new LoggedTalonFX(12, pivotTable, "PivotLeft");
        rightMotor = new LoggedTalonFX(13, pivotTable, "PivotRight");

        configMotor();

        // Logging
        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
        System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

        this.log();

        register();
    }

    /**
     * configs both of the talonFX with the
     * <ol>
     * <li>NeutralMode</li>
     * <li>gearRatio</li>
     * <li>SoftwareLimitSwitch</li>
     * <li>motion magic values/PID values</li>
     * <li>curet limit</li>
     * <li>inversion</li>
     * <li>and zeros to the intake pose</li>
     * </ol>
     */
    public void configMotor() {
        TalonFXConfiguration pivotConf = new TalonFXConfiguration();

        pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConf.Feedback.SensorToMechanismRatio = 100;
        pivotConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConf.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConf.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;

        // pivotConf.Slot0.kP = 13;
        // pivotConf.Slot0.kG = 0.27;
        // pivotConf.Slot0.kV = 1.88;
        // pivotConf.Slot0.kA = 0.01;

        pivotConf.Slot0.kP = 163;
        pivotConf.Slot0.kD = 10;
        pivotConf.Slot0.kG = 0.07; // TODO: tune PID

        pivotConf.MotionMagic.MotionMagicAcceleration = 0.75;
        pivotConf.MotionMagic.MotionMagicCruiseVelocity = 2;

        // TODO: Do the math for the current limiting
        pivotConf.CurrentLimits.SupplyCurrentLimit = 75;
        pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(pivotConf);

        pivotConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(pivotConf);

        MotionMagicCtrlMode.FeedForward = 0;

        zeroToIntakePose();
    }

    // ---------- Generic Functions ----------
    /**
     * @return true if the pivot is aimed, within deadband, to the last set rotation
     *         setpoint.
     */
    public boolean isAimed() {
        return isAimedTo(MotionMagicCtrlMode.Position);
    }

    public boolean isAimedTo(double rotations) {
        double error = leftMotor.getPosition().getValueAsDouble() - rotations;
        return Math.abs(error) < defaultDeadband;
    }

    public void currentZeroingSequence() {
        // TODO: make currentZeroingSequence
        zeroToIntakePose();
    }

    /**
     * Zeroes the current pivot position to the value it should be at if it is at
     * intake position.
     * <p>
     * THE INTAKE MUST BE IN ITS INTAKE POSITION
     */
    public void zeroToIntakePose() {
        leftMotor.setPosition(intakePosition);
        rightMotor.setPosition(intakePosition);
    }

    private void disableMotors() {
        leftMotor.disable();
        rightMotor.disable();
    }

    /**
     * Controls the pivot based on a Rotation. Used for automated
     * control of setting angles
     * 
     * @param rotations [-0.07161, .25] the rotation to set the pivot at
     *                  positive moves in the direction of amp pose
     */
    // public void setPosition(double rotations) {
    // if (rotations > ampPosition || rotations < intakePosition) {
    // DriverStation.reportWarning(
    // "setting pivot position outside of reachable range, THIS COULD DAMAGE THE
    // ROBOT", true);
    // }
    // MotionMagicCtrlMode.Position = rotations;

    // leftMotor.setControl(MotionMagicCtrlMode);
    // rightMotor.setControl(MotionMagicCtrlMode);
    // }

    // ---------- Command CtrlModes ----------
    /**
     * Controls the pivot based on a DutyCycle supplier. Used for manual
     * controller-based override control.
     * 
     * @param dutyCycle [-1, 1] the speed to control the pivot at,
     *                  max speed based on outputMax,
     *                  positive moves in the direction of amp pose
     * 
     * @return a command that requires the pivot and when on ends the motors are
     *         disabled
     */
    public Command dutyCycleCtrl(DoubleSupplier dutyCycle) {
        Command cmd = new Command() {
            public void execute() {
                if (dutyCycle.getAsDouble() > 1 || dutyCycle.getAsDouble() < -1) {
                    DriverStation.reportWarning(
                            "setting pivot dutyCycle outside of controllable range", true);
                }
                DutyCycleCtrlMode.Output = -dutyCycle.getAsDouble() * dutyCycleMax.getAsDouble();

                leftMotor.setControl(DutyCycleCtrlMode);
                rightMotor.setControl(DutyCycleCtrlMode);
            }

            public void end(boolean interrupted) {
                leftMotor.disable();
                rightMotor.disable();
            }
        };

        cmd.addRequirements(this);
        return cmd;
    }

    public void MMPositionCtrl(double rotations) {
        if (rotations > ampPosition || rotations < intakePosition) {
            DriverStation.reportWarning(
                    "setting pivot position outside of reachable range, THIS COULD DAMAGE THE ROBOT", true);
        }
        MotionMagicCtrlMode.Position = rotations;

        leftMotor.setControl(MotionMagicCtrlMode);
        rightMotor.setControl(MotionMagicCtrlMode);
    }

    /**
     * Controls the pivot based on a Rotation. Used for automated
     * control of setting angles
     * 
     * @param rotations [-0.07161, .25] the rotation to set the pivot at
     *                  positive moves in the direction of amp pose
     * 
     * @return a command that requires the pivot and when on ends the motors are
     *         disabled
     */
    public Command MMPositionCmd(double rotations) {
        Command cmd = new Command() {
            @Override
            public void initialize() {
                MMPositionCtrl(rotations);
            }

            @Override
            public boolean isFinished() {
                return isAimed();
            }
        };

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

        cmd.addRequirements(this);
        return cmd;
    }

    // ---------- Commands ----------

    /**
     * Moves the pivot to the position required to shoot into the speaker from the
     * subwoofer.
     * 
     * @return a command that requires the pivot and when on ends the motors are
     *         disabled
     */
    public Command toSubwoofer() {
        Command cmd = MMPositionCmd(subwooferPosition);

        return cmd;
    }

    /**
     * Moves the pivot to the position required to shoot into the amp.
     * 
     * @return a command that requires the pivot and when it ends the motors are
     *         disabled
     */
    public Command toAmp() {
        Command cmd = MMPositionCmd(ampPosition);

        return cmd;
    }

    /**
     * Moves the pivot to the position required to Intake a note
     * 
     * @return a command that requires the pivot and when it ends the motors are
     *         disabled
     */
    public Command toIntakeUntilAimed() {
        return MMPositionCmd(intakePosition);
    }

    // ---------- Logging ----------
    public void log() {
        leftMotor.log();
        rightMotor.log();
        setpoint.set(MotionMagicCtrlMode.Position);
        actualRotation.set(leftMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void close() {
        // Hardware
        leftMotor.close();
        rightMotor.close();

        // Network Table
        dutyCycleMax.close();

        setpoint.close();
        actualRotation.close();
    }
}
