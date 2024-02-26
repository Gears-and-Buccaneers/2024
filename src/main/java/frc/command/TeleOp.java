package frc.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hardware.Controller;
import frc.system.Drivetrain;

public class TeleOp extends Command {
	final Drivetrain drivetrain;

	final double maxVelocity;
	final double maxRotVel;

	final Controller.Axis x;
	final Controller.Axis y;
	final Controller.Axis theta;

	// final Controller.Button robotRel;

	public TeleOp(Drivetrain drivetrain, double maxVelocity, double maxRotVel,
			Controller.Axis x, Controller.Axis y, Controller.Axis theta
	// Controller.Button robotRel
	) {
		addRequirements(drivetrain);

		this.drivetrain = drivetrain;

		this.maxVelocity = maxVelocity;
		this.maxRotVel = maxRotVel;

		this.x = x;
		this.y = y;
		this.theta = theta;

		// this.robotRel = robotRel;
	}

	@Override
	public void execute() {
		// ChassisSpeeds speed;

		Translation2d xy = new Translation2d(x.get(), y.get());

		double velocity = Math.min(xy.getNorm(), 1) * maxVelocity;
		Rotation2d direction = xy.getAngle();

		xy = new Translation2d(velocity, direction);

		// Rotation2d rot = drivetrain.pose().getRotation();

		drivetrain.drive(new Transform2d(xy, new Rotation2d(theta.get() * maxRotVel)));
	}
}
