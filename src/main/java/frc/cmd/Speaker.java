package frc.cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Pivot;
import frc.system.Swerve;

public class Speaker extends Command {
	private final Swerve drivetrain;
	private final Pivot pivot;

	public Speaker(Swerve drivetrain, Pivot pivot) {
		this.drivetrain = drivetrain;
		this.pivot = pivot;

		addRequirements(drivetrain, pivot);
	}

	@Override
	public void execute() {
		
	}
}
