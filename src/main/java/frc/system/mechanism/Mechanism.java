package frc.system.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.system.mechanism.components.Intake;
import frc.system.mechanism.components.Pivot;
import frc.system.mechanism.components.Shooter;
import frc.system.mechanism.components.Transit;

public class Mechanism implements frc.system.Mechanism {
	final Intake intake;
	final Transit transit;
	final Pivot pivot;
	final Shooter shooter;

	public Mechanism(Intake intake, Transit transit, Pivot pivot, Shooter shooter) {
		this.intake = intake;
		this.transit = transit;
		this.pivot = pivot;
		this.shooter = shooter;
	}

	@Override
	public Command intake() {
		Command cmd = pivot.toIntake().andThen(intake.run().raceWith(transit.run()));
		cmd.addRequirements(this);
		return cmd;
	}

	@Override
	public Command aim() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Command shoot() {
		Command cmd = shooter.run().raceWith(shooter.waitPrimed().andThen(transit.run()));
		cmd.addRequirements(this);
		return cmd;
	}
}
