package frc.system;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Pivot implements Subsystem {
	public final Translation3d origin = new Translation3d(Units.inchesToMeters(27 / 2 - 4), 0,
			Units.inchesToMeters(21.75));

	/**
	 * The shooter degree offset from being perpendicular to the arm, in radians.
	 */
	private final double armOffsetRad;
	private final double exitDistance;

	private final MotionMagicDutyCycle positionCtrl = new MotionMagicDutyCycle(0);

	private final double defaultDeadband;
	/** The position for intaking, in rotations. */
	private final double intakePosition;
	/** The position for shooting into the amp, in rotations. */
	private final double ampPosition;

	private final String simpleName = this.getClass().getSimpleName();

	// Hardware
	private TalonFX leftMotor;
	private TalonFX rightMotor;

	// Network
	private NetworkTable table;

	private DoubleSubscriber outputMax;

	private final double subwooferPosition;

	/**
	 * Assumes 90 degree angle is 0 and the intake position has negative rotational
	 * sign.
	 */
	public Pivot(NetworkTable networkTable) {
		double armLength = 0.43;
		armOffsetRad = Units.degreesToRadians(52);

		exitDistance = armLength * Math.sin(armOffsetRad);

		defaultDeadband = 0.01;

		intakePosition = -7.161 / 100;
		ampPosition = 0.25;
		subwooferPosition = -0.0530455469;

		table = networkTable.getSubTable(simpleName);

		leftMotor = new TalonFX(12);
		rightMotor = new TalonFX(13);

		configPID();
        register();
	}

	public void configPID() {
		this.table.getDoubleTopic("speed").publish();
		outputMax = table.getDoubleTopic("speed").subscribe(0.3);
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

		pivotConf.Slot0.kP = 13;
		pivotConf.Slot0.kG = 0.07; // TODO: check this val

		// TODO: pick a number
		pivotConf.CurrentLimits.SupplyCurrentLimit = 75;
		pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;

		pivotConf.MotionMagic.MotionMagicAcceleration = 0.5;
		pivotConf.MotionMagic.MotionMagicCruiseVelocity = 1;

		pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		leftMotor.getConfigurator().apply(pivotConf);
		pivotConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rightMotor.getConfigurator().apply(pivotConf);

		zeroToIntake();
	}

	/**
	 * Controls the pivot based on position closed-loop to the specified number of
	 * rotations.
	 */
	private void setSetpoint(double rotations) {
		positionCtrl.Position = rotations;

		leftMotor.setControl(positionCtrl);
		rightMotor.setControl(positionCtrl);
	}

	/**
	 * Controls the pivot based on a velocity supplier. Used for manual
	 * controller-based override control.
	 */
	public Command velocity(DoubleSupplier output) {
		Command cmd = new Command() {
			public void execute() {
				DutyCycleOut ctrl = new DutyCycleOut(-output.getAsDouble() * outputMax.getAsDouble());

				leftMotor.setControl(ctrl);
				rightMotor.setControl(ctrl);
			}

			@Override
			public void end(boolean interrupted) {
				leftMotor.disable();
				rightMotor.disable();
			}
		};

		cmd.addRequirements(this);
		return cmd;
	}

	/** Returns a command which goes to the specified position, then completes. */
	public Command toPosition(double rotations) {
		Command cmd = new Command() {
			@Override
			public void initialize() {
				setSetpoint(rotations);
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
	 * Moves the pivot to the position required to shoot into the speaker from the
	 * subwoofer.
	 */
	public Command subwoofer() {
		return toPosition(subwooferPosition);
	}

	/**
	 * Moves the pivot to the position required to shoot into the amp.
	 */
	public Command amp() {
		return toPosition(ampPosition);
	}

	/** Moves the pivot to intaking position. */
	public Command intake() {
		return toPosition(intakePosition);
	}

	/**
	 * Continuously aims at a point `distance` meters away from the pivot origin,
	 * with an angle of elevation of `pitch` radians.
	 */
	public Command aimAt(double distance, double pitch) {
		Command cmd = new Command() {
			@Override
			public void execute() {
				double rotations = Units.radiansToRotations(armOffsetRad + Math.asin(exitDistance / distance) - pitch);
				setSetpoint(rotations);
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
	 * Returns true if the pivot is aimed, within deadband, to the last set rotation
	 * setpoint.
	 */
	public boolean isAimed() {
		return Math.abs(leftMotor.getPosition().getValueAsDouble() - positionCtrl.Position) < defaultDeadband;
	}

	/**
	 * Zeroes the current pivot position to the value it should be at if it is at
	 * intake position.
	 */
	public void zeroToIntake() {
		leftMotor.setPosition(intakePosition);
		rightMotor.setPosition(intakePosition);
	}
}
