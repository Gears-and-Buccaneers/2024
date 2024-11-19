package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    // Hardware
    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;

    private double defaultIntakeSpeed = .8;

    public Intake() {
        // Motors
        leftMotor = new TalonSRX(9);
        rightMotor = new TalonSRX(10);

        // Configs
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();

        currentLimits.currentLimit = 40;
        currentLimits.enable = true;

        leftMotor.configSupplyCurrentLimit(currentLimits);
        rightMotor.configSupplyCurrentLimit(currentLimits);

		register();
    }

    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? defaultIntakeSpeed : -defaultIntakeSpeed;
                speed *= percentMaxSpeed;

                leftMotor.set(TalonSRXControlMode.PercentOutput, speed);
                rightMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }

    private void disable() {
        leftMotor.set(TalonSRXControlMode.Disabled, 0);
        rightMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    /**
     * returns a command that runs the intake FORWARD
     * when it ends it disables the intake
     * intake command requires itself
     */
    public Command runIn() {
        return feed(true, 1);
    }

    /**
     * returns a command that runs the intake BACKWARD
     * when it ends it disables the intake
     * intake command requires itself
     */
    public Command runOut() {
        return feed(false, 1);
    }
}
