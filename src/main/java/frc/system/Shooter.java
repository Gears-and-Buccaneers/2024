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

    private double hi = 0.8;
    private double lo = 0.2;

    public Shooter(int id) {
        // Motor
        shooterMotor = new TalonSRX(id);

        // Configs
        shooterMotor.setInverted(false);

        shooterMotor.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();

        currentLimits.currentLimit = 40;
        currentLimits.enable = true;

        shooterMotor.configSupplyCurrentLimit(currentLimits);

		register();
    }

    private Command feed(double speed) {
        Command cmd = new Command() {
            public void initialize() {
                shooterMotor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                shooterMotor.set(TalonSRXControlMode.Disabled, 0);
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }

    public Command runHi() {
        return feed(hi);
    }

    public Command runLo() {
        return feed(lo);
    }

    /*
     * Runs the shooter to the high speed, lowering to the low speed on end.
     */
    public Command runHiInterrupt() {
        Command cmd = new Command() {
            public void initialize() {
                shooterMotor.set(TalonSRXControlMode.PercentOutput, hi);
            }
            
            public void end(boolean interrupted) {
                shooterMotor.set(TalonSRXControlMode.PercentOutput, lo);
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }
}
