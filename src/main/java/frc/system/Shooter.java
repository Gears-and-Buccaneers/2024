package frc.system;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter implements Subsystem {
	private final String simpleName = this.getClass().getSimpleName();

	// Hardware
	private TalonFX leftMotor;
	private TalonFX rightMotor;

	// Network
	private NetworkTable Table;
	/** Units: RPM */
	private DoubleSubscriber shooterSpeed;
	private DoubleSubscriber shooterSpeedDeadBand;

	final VelocityDutyCycle m_request = new VelocityDutyCycle(0);

	public Shooter(NetworkTable networkTable) {
		this.Table = networkTable.getSubTable(simpleName);

		// Motors
		leftMotor = new TalonFX(14);
		rightMotor = new TalonFX(15);

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

		// TODO: CONFIG and CurrentLimit
		// TODO: make sure that phinox 6 imple is corect

		// Vars
		shooterSpeed = Table.getDoubleTopic("shooterSpeed").subscribe(5000);
		this.Table.getDoubleTopic("shooterSpeed").publish();

		shooterSpeedDeadBand = Table.getDoubleTopic("shooterSpeedDeadBand").subscribe(5000);
		this.Table.getDoubleTopic("shooterSpeedDeadBand").publish();

		System.out.println("[Init] Creating " + simpleName + " with:");
		System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
		System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

		this.log();

		register();

		setDefaultCommand(stop());
	}

	private void runForward(double speed) {
		DutyCycleOut output = new DutyCycleOut(speed / 6000);

		leftMotor.setControl(output);
		rightMotor.setControl(output);
	}

	public void disable() {
		leftMotor.disable();
		rightMotor.disable();
	}

	// Commands
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
		Command cmd = new Command() {
			public void initialize() {
				runForward(shooterSpeed.getAsDouble());
			}
		};

		cmd.addRequirements(this);
		return cmd;
	}

	public Command shootAmp() {
		Command cmd = new Command() {
			public void initialize() {
				runForward(1000);
			}
		};

		cmd.addRequirements(this);
		return cmd;
	}

	public Command reverse() {
		Command cmd = new Command() {
			public void initialize() {
				runForward(-100);
			}
		};

		cmd.addRequirements(this);
		return cmd;
	}

	public Command waitPrimed() {
		// TODO: acceleration deadbanding
		// leftMotor.getAcceleration();

		// boolean leftMotorAtSpeed = Math.abs(
		// leftMotor.getRotorVelocity().getValue() - shooterSpeed.getAsDouble()) <=
		// shooterSpeedDeadBand
		// .getAsDouble();
		// boolean rightMotorAtSpeed = Math.abs(
		// rightMotor.getRotorVelocity().getValue() - shooterSpeed.getAsDouble()) <=
		// shooterSpeedDeadBand
		// .getAsDouble();

		// return new WaitUntilCommand(() -> {
		// return leftMotorAtSpeed && rightMotorAtSpeed;
		// });
		return new WaitCommand(1);
	}

	// Logging
	public void log() {
		Table.getStringArrayTopic("ControlMode").publish()
				.set(new String[] {
						leftMotor.getControlMode().toString(),
						rightMotor.getControlMode().toString() });
		Table.getIntegerArrayTopic("DeviceID").publish()
				.set(new long[] {
						leftMotor.getDeviceID(),
						rightMotor.getDeviceID() });

		Table.getDoubleArrayTopic("set speed").publish()
				.set(new double[] {
						leftMotor.get(),
						rightMotor.get() });

		Table.getDoubleArrayTopic("getVelocity").publish()
				.set(new double[] {
						leftMotor.getVelocity().getValueAsDouble(),
						rightMotor.getVelocity().getValueAsDouble() });

		Table.getDoubleArrayTopic("getDeviceTemp").publish()
				.set(new double[] {
						leftMotor.getDeviceTemp().getValueAsDouble(),
						rightMotor.getDeviceTemp().getValueAsDouble() });

		Table.getDoubleArrayTopic("getMotorVoltage").publish()
				.set(new double[] {
						leftMotor.getMotorVoltage().getValueAsDouble(),
						rightMotor.getMotorVoltage().getValueAsDouble() });

		// Table.getDoubleArrayTopic("Temp").publish()
		// .set(new double[] { leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp() });
		// Table.getDoubleArrayTopic("Supply Current").publish()
		// .set(new double[] { leftMotor.getSupplyCurrent(),
		// rightMotor.getSupplyCurrent() });
		// Table.getDoubleArrayTopic("Stator Current").publish()
		// .set(new double[] { leftMotor.getStatorCurrent(),
		// rightMotor.getStatorCurrent() });
		// Table.getDoubleArrayTopic("Motor Voltage").publish()
		// .set(new double[] { leftMotor.getMotorVoltage(), rightMotor.getMotorVoltage()
		// });
	}
}
