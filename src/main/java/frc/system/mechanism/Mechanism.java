package frc.system.mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Main;
import frc.command.TeleOp;
import frc.system.Drivetrain;
import frc.system.mechanism.components.Intake;
import frc.system.mechanism.components.Pivot;
import frc.system.mechanism.components.Shooter;
import frc.system.mechanism.components.Transit;

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

		Command toIntake = pivot.toIntake();

		toIntake.addRequirements(this);

		setDefaultCommand(toIntake);
	}

	@Override
	public Command intake() {
		Command cmd = pivot.toIntake().andThen(intake.run().raceWith(transit.run()));
		cmd.addRequirements(this);
		return cmd;
	}

	@Override
	public Command aim(TeleOp teleop) {
		Drivetrain drivetrain = teleop.drivetrain;

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

				Translation2d xy = teleop.translation();

				drivetrain.driveFacing(xy.getX(), xy.getY(), new Rotation2d(yaw));
				pivot.aim(pitch, distance);
			}
		};

		cmd.addRequirements(this, teleop.drivetrain);
		return cmd;
	}

	@Override
	public Command shoot() {
		Command cmd = shooter.run().raceWith(shooter.waitPrimed().andThen(transit.run()));
		cmd.addRequirements(this);
		return cmd;
	}
}
