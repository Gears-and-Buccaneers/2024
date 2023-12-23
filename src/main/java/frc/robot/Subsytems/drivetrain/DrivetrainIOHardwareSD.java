package frc.robot.Subsytems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.sensor.imu.*;

public class DrivetrainIOHardwareSD implements DrivetrainRequirments {
    public IMU imu;

    public SwerveDriveOdometry odometry;

    /** FL, FR, BL, BR */
    public Module[] modules;

    public double maxSpeed = 4.0;
    public double maxAcceleration = 3.0;
    public double maxAngularVelocity = 8.0;
    public double maxAngularAcceleration = 4.0;
    /** meaters from center fathest out point */
    public double drivetrainRadius = 0.4;

    

    /** Locations of the wheels relative to the robot center. FL, FR, BL, BR */
    Translation2d[] wheelLocations = {
            new Translation2d(0.265, 0.265),
            new Translation2d(0.265, -0.265),
            new Translation2d(-0.265, 0.265),
            new Translation2d(-0.265, -0.265)
    };

    public DrivetrainIOHardwareSD(SubsystemBase subsystemBase) {
        System.out.println("[Init] Creating " + this.getClass().getSimpleName());

        //IMU configs
        imu = new Pigeon(0);
        //Init Prefrences
        Preferences.initDouble("maxSpeed", maxSpeed);
    }

    // Chassis Speeds ------------------------
    @Override
    public ChassisSpeeds getChassisSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeed'");
    }

    @Override
    public void setChassisSpeed(ChassisSpeeds speeds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setChassisSpeed'");
    }

    @Override
    public void stopChassis() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopChassis'");
    }

    // odometry-------------------------------------------
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        // odometry.reset(imu.getYaw(), getDrivetrainState(), pose);
    }

    @Override
    public void updateOdometry(Rotation2d yaw, double state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateOdometry'");
    }

    @Override
    public void zeroGyro() {
        imu.zeroIMU();
    }

    @Override
    public void configAutoBuilder(SubsystemBase subsystemBase) {
        AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
                // pose)
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                        // your
                        // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        maxSpeed, // Max module speed, in m/s
                        drivetrainRadius, // Drive base radius in meters. Distance from robot center to furthest
                        // module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options
                // here
                ),
                subsystemBase // Reference to this subsystem to set requirements
        );
    }

    // auto sendable and auto closable
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        maxSpeed = Preferences.getDouble("maxSpeed", maxSpeed);
        // if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
        //     m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
        //     m_controller.setP(m_armKp);
        // }
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        inputs.GyroConnected = imu.connected();
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}
