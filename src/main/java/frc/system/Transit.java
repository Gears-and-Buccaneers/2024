package frc.system;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Transit implements Subsystem {
    // Hardware
    private final TalonSRX motor;

    private final double defaultSpeed = .8;

    public Transit(int id) {
        // Motors
        motor = new TalonSRX(id);

        // Configs
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Brake);

        SupplyCurrentLimitConfiguration currentLimits = new SupplyCurrentLimitConfiguration();

        currentLimits.currentLimit = 40;
        currentLimits.enable = true;

        motor.configSupplyCurrentLimit(currentLimits);

		register();
    }

    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? defaultSpeed : -defaultSpeed;
                speed *= percentMaxSpeed;

                motor.set(TalonSRXControlMode.PercentOutput, speed);
            }

            public void end(boolean interrupted) {
                disable();
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }

    private void disable() {
        motor.set(TalonSRXControlMode.Disabled, 0);
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
