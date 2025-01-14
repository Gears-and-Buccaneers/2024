package frc.system;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Transit implements Subsystem {
    // Hardware
    private final TalonFX motor;

    private final double defaultSpeed = -0.1;

    public Transit(int id) {
        // Motors
        motor = new TalonFX(id);

        // Configs
        motor.setInverted(false);

		register();
    }

    private Command feed(boolean forwards, double percentMaxSpeed) {
        Command cmd = new Command() {
            public void initialize() {
                double speed = forwards ? defaultSpeed : -defaultSpeed;
                speed *= percentMaxSpeed;

                motor.setControl(new DutyCycleOut(speed));
            }

            public void end(boolean interrupted) {
                disable();
            }
        };

        cmd.addRequirements(this);

        return cmd;
    }

    private void disable() {
        motor.setControl(new NeutralOut());
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
