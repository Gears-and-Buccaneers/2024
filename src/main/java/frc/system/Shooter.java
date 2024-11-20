package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem {
    // Hardware
    private final TalonSRX shooterMotor;

    private double defaultShooterSpeed = .8;

    public Shooter(int id) {
        // Motor
        shooterMotor = new TalonSRX(int id);

        // Configs
        shooterMotor.setInverted(false);

        shooterMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();

        currentLimits.currentLimit = 40;
        currentLimits.enable = true;

        shooterMotor.configSupplyCurrentLimit(currentLimits);

		register();
    }

    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? defaultShooterSpeed : -defaultShooterSpeed;
                speed *= percentMaxSpeed;

                shooterMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }

    private void disable() {
        shooterMotor.set(TalonSRXControlMode.Disabled, 0);
    }

    /**
     * returns a command that runs the Shooter FORWARD
     * when it ends it disables the Shooter
     * Shooter command requires itself
     */
    public Command runIn() {
        return feed(true, 1);
    }

    /**
     * returns a command that runs the Shooter SLOWER
     * when it ends it disables the Shooter
     * Shooter command requires itself
     */
    public Command runOut() {
        return feed(true, .8);
    }
}
