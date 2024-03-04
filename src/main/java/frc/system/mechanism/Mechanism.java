package frc.system.mechanism;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Main;
import frc.system.Drivetrain;
import frc.system.mechanism.components.Intake;
import frc.system.mechanism.components.Pivot;
import frc.system.mechanism.components.Shooter;
import frc.system.mechanism.components.Transit;

/*

aim
pivot
intake


 */

public class Mechanism implements frc.system.Mechanism {
	final Translation3d originToMechanism;

	final Intake intake;
	final Transit transit;
	final Pivot pivot;
	final Shooter shooter;

	public Mechanism(Translation3d originToMechanism, Intake intake, Transit transit, Pivot pivot, Shooter shooter) {
		this.originToMechanism = originToMechanism;

		this.intake = intake;
		this.transit = transit;
		this.pivot = pivot;
		this.shooter = shooter;

		// TODO: reenable this when motor is tuned
		// Command toIntake = pivot.toIntake();
		// toIntake.addRequirements(this);
		// setDefaultCommand(toIntake);
	}

	@Override
	public void tmp_zeroPivot() {
		pivot.motor.setPosition(0);
	}

	@Override
	public void tmp_runPivot(double output) {
		pivot.motor.setPercent(output * 0.15);
	}

	@Override
	public Command intake() {
		Command cmd = intake.run().raceWith(transit.intake());
		// Command cmd =
		// pivot.toIntake().andThen(intake.run().raceWith(transit.intake()));
		cmd.addRequirements(this);
		return cmd;
	}

	@Override
	public Command amp() {
		Command cmd = shooter.run().alongWith(shooter.waitPrimed().andThen(transit.shoot()));
		// Command cmd = pivot.toAmp().alongWith(shooter.run());
		cmd.addRequirements(this);
		return cmd;
	}

	@Override
	public Command speaker(Drivetrain drivetrain, Supplier<Translation2d> translation) {
		Command cmd = new Command() {
			@Override
			public void execute() {
				Pose2d robotPose = drivetrain.pose();

				Translation3d origin = new Translation3d(robotPose.getX(), robotPose.getY(), 0);
				Translation3d mechanism = origin.plus(originToMechanism);
				Translation3d vector = Main.speakerPosition.minus(mechanism);

				double distance = vector.getNorm();
				double pitch = Math.asin(vector.getZ() / distance);
				double yaw = Math.atan2(vector.getY(), vector.getX());

				Translation2d xy = translation.get();

				drivetrain.driveFacing(xy.getX(), xy.getY(), new Rotation2d(yaw));
				pivot.aim(pitch, distance);
			}
		}.alongWith(shooter.run());

		cmd.addRequirements(this, drivetrain);
		return cmd;
	}

	@Override
	public Command shoot() {
		Command cmd = shooter.waitPrimed().andThen(transit.shoot());
		cmd.addRequirements(this);
		return cmd;
	}
}
