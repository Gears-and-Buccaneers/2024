package frc.system.mechanism.components;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.hardware.ProfiledMotor;
import frc.hardware.profiledmotor.FX;
import frc.system.mechanism.MechanismReq;

public class Shooter implements MechanismReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private ProfiledMotor leftMotor;
    private ProfiledMotor rightMotor;

    // Network
    private NetworkTable Table;
    /** Units: RPM */
    private DoubleSubscriber shooterSpeed;
    private DoubleSubscriber shooterSpeedDeadBand;

    // vars
    private BooleanSupplier transitHasNote;

    public Shooter(NetworkTable networkTable, BooleanSupplier feederNote) {
        this.Table = networkTable.getSubTable(simpleName);

        // Motors
        leftMotor = new FX(9);
        rightMotor = new FX(10);
        // TODO: CONFIG and CurrentLimit

        // Vars
        shooterSpeed = Table.getDoubleTopic("shooterSpeed").subscribe(6000);
        this.Table.getDoubleTopic("shooterSpeed").publish();

        shooterSpeedDeadBand = Table.getDoubleTopic("shooterSpeedDeadBand").subscribe(6000);
        this.Table.getDoubleTopic("shooterSpeedDeadBand").publish();

        this.transitHasNote = feederNote;

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + leftMotor.getClass().getSimpleName() + " ID:" + leftMotor.getDeviceID());
        System.out.println("\t" + rightMotor.getClass().getSimpleName() + " ID:" + rightMotor.getDeviceID());

        this.log();
    }

    public void periodic() {
        this.log();
    }

    private void runForward(boolean forwards) {
        double speed = forwards ? shooterSpeed.get() : -shooterSpeed.get();

        leftMotor.setVelocity(speed);
        rightMotor.setVelocity(speed);
    }

    public void disable() {
        leftMotor.setPercent(0);
        rightMotor.setPercent(0);
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

    public Command shoot() {
        return new Command() {
            public void initialize() {
                runForward(true);
            }

            public boolean isFinished() {
                return !transitHasNote.getAsBoolean();
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

    public Command waitPrimed() {
        boolean leftMotorAtSpeed = Math.abs(leftMotor.velocity() - shooterSpeed.getAsDouble()) <= shooterSpeedDeadBand
                .getAsDouble();
        boolean rightMotorAtSpeed = Math.abs(rightMotor.velocity() - shooterSpeed.getAsDouble()) <= shooterSpeedDeadBand
                .getAsDouble();
        ;

        return new WaitUntilCommand(() -> {
            return leftMotorAtSpeed && rightMotorAtSpeed;
        });
    }

    // Logging
    public void log() {
        Table.getStringArrayTopic("ControlMode").publish()
                .set(new String[] { leftMotor.getControlMode().toString(), rightMotor.getControlMode().toString() });
        Table.getIntegerArrayTopic("DeviceID").publish()
                .set(new long[] { leftMotor.getDeviceID(), rightMotor.getDeviceID() });

        Table.getDoubleArrayTopic("Temp").publish()
                .set(new double[] { leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp() });
        Table.getDoubleArrayTopic("Supply Current").publish()
                .set(new double[] { leftMotor.getSupplyCurrent(), rightMotor.getSupplyCurrent() });
        Table.getDoubleArrayTopic("Stator Current").publish()
                .set(new double[] { leftMotor.getStatorCurrent(), rightMotor.getStatorCurrent() });
        Table.getDoubleArrayTopic("Motor Voltage").publish()
                .set(new double[] { leftMotor.getMotorVoltage(), rightMotor.getMotorVoltage() });
    }

    public void close() throws Exception {
    }
}
