package frc.robot.Subsytems.drivetrain;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSub extends SubsystemBase implements AutoCloseable{
    public DrivetrainSub() {
        
    }

    @Override
    public void periodic() {
        if (imu != null) {
            System.out.println("imu: " + imu.getYaw());
            odometry.update(imu.getYaw(), getDrivetrainState());
        }

        var pose = odometry.getPoseMeters();
        field.setRobotPose(pose);

        System.out.println("pose: " + pose.toString());

        SmartDashboard.putString("Chassie Speed Calcualted", getChassisSpeed().toString());
        // SmartDashboard.putData(imu);
        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("Odometry yaw", pose.getRotation().getDegrees());

        for (ModuleBase module : modules) {
            SmartDashboard.putData("Mod" + module.number + ": State", module);
        }
    }

       /**
     * drives the drivetrain
     *
     * @param translation      the x and y compnet of the movment (meaters/second)
     * @param rotation         the rotation of the robot (radians/second)
     * @param notFieldRelative false for field relitive
     */
    
    public Command drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, imu.getYaw());

        setChassisSpeed(speeds);

        return run(() -> {System.out.println(1)});
    }
}