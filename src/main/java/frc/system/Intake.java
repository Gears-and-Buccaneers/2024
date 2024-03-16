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
	private TalonSRX leftMotor;
	private TalonSRX rightMotor;

	// Network
	private NetworkTable Table;
	private DoubleSubscriber intakeSpeed;

	public Intake(NetworkTable networkTable) {
		this.Table = networkTable.getSubTable(simpleName);

		// Motors
		leftMotor = new TalonSRX(9);
		rightMotor = new TalonSRX(10);

		leftMotor.setInverted(true);
		rightMotor.setInverted(true);

		leftMotor.setNeutralMode(NeutralMode.Coast);
		rightMotor.setNeutralMode(NeutralMode.Coast);

		SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();
		currentLimits.currentLimit = 50;
		currentLimits.enable = true;

		leftMotor.configSupplyCurrentLimit(currentLimits);
		rightMotor.configSupplyCurrentLimit(currentLimits);

		// Vars
		intakeSpeed = Table.getDoubleTopic("intakeSpeed").subscribe(.8);
		this.Table.getDoubleTopic("intakeSpeed").publish();

		System.out.println("[Init] Creating " + simpleName + " with:");
		System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
		System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

		this.log();
	}

	private void runForward(boolean forwards) {
		double speed = forwards ? intakeSpeed.get() : -intakeSpeed.get();
		System.out.println(1);
		leftMotor.set(TalonSRXControlMode.PercentOutput, speed);
		rightMotor.set(TalonSRXControlMode.PercentOutput, -speed);
	}

	public void disable() {
		leftMotor.set(TalonSRXControlMode.Disabled, 0);
		rightMotor.set(TalonSRXControlMode.Disabled, 0);
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
				.set(new String[] { leftMotor.getControlMode().toString(), rightMotor.getControlMode().toString() });
		Table.getIntegerArrayTopic("DeviceID").publish()
				.set(new long[] { leftMotor.getDeviceID(), rightMotor.getDeviceID() });

		Table.getDoubleArrayTopic("Temp").publish()
				.set(new double[] { leftMotor.getTemperature(), rightMotor.getTemperature() });
		Table.getDoubleArrayTopic("Supply Current").publish()
				.set(new double[] { leftMotor.getSupplyCurrent(), rightMotor.getSupplyCurrent() });
		Table.getDoubleArrayTopic("Stator Current").publish()
				.set(new double[] { leftMotor.getStatorCurrent(), rightMotor.getStatorCurrent() });
		Table.getDoubleArrayTopic("Output Voltage").publish()
				.set(new double[] { leftMotor.getMotorOutputVoltage(), rightMotor.getMotorOutputVoltage() });
		Table.getDoubleArrayTopic("Bus Voltage").publish()
				.set(new double[] { leftMotor.getBusVoltage(), rightMotor.getBusVoltage() });
	}

	public void close() throws Exception {
	}

}
