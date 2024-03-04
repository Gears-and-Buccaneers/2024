package frc.system;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Mechanism extends Subsystem {
	void tmp_zeroPivot();

	void tmp_runPivot(double output);

	Command intake();

	Command amp();

	Command speaker(Drivetrain drivetrain, Supplier<Translation2d> translation);

	Command shoot();
}
