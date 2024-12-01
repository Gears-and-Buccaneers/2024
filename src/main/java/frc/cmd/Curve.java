package frc.cmd;

import java.util.function.DoubleFunction;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.system.Swerve;

public class Curve extends Command {
    final Swerve swerve;
    final DoubleFunction<Translation2d> deriv;
    final FieldCentricFacingAngle req = new FieldCentricFacingAngle();
    double t;

    public Curve(Swerve swerve, DoubleFunction<Translation2d> deriv) {
        this.swerve = swerve;
        this.deriv = deriv;

        addRequirements(swerve);
    }

    double time() {
        return RobotController.getFPGATime() / 1_000_000.0;
    }

    @Override
    public void initialize() {
        t = time();
    }

    @Override
    public void execute() {
        double dt = time() - t;

        Translation2d vec = deriv.apply(dt);

        req.VelocityX = vec.getX();
        req.VelocityY = vec.getY();

        req.TargetDirection = vec.getAngle();

        swerve.setControl(req);
    }
}
