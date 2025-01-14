package frc.cmd;

import java.util.function.DoubleFunction;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Swerve;

public class Curve extends Command {
    final Swerve swerve;
    final DoubleFunction<Translation2d> deriv;
    final FieldCentricFacingAngle req = new FieldCentricFacingAngle();
    double t = 0; 

    public Curve(Swerve swerve, DoubleFunction<Translation2d> deriv) {
        this.swerve = swerve;
        this.deriv = deriv;

        req.HeadingController = new PhoenixPIDController(3, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        t = 0;
    }

    @Override
    public void execute() {
        Translation2d vec = deriv.apply(t);
        t += 0.02;

        req.VelocityX = vec.getX();
        req.VelocityY = vec.getY();

        req.TargetDirection = vec.getAngle().rotateBy(Rotation2d.fromDegrees(180));

        swerve.setControl(req);
    }
}
