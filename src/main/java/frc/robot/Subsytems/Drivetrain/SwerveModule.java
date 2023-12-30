package frc.robot.Subsytems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.hardware.Motors.Motor;
import frc.lib.hardware.Motors.PID.*;
import frc.lib.hardware.sensor.encoders.REVBoreEncoder;
import frc.robot.Subsytems.SubsytemRequirments;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class SwerveModule implements SubsytemRequirments {
  private final Motor mDrive;
  private final Motor mSteer;

  private final String name;

  public SwerveModule(int driveId, int steerId, String name) {
    mDrive = new Motor(Motor.ControllerType.TallonSRX, driveId, Motor.Type.Falcon500);
    mSteer = new Motor(Motor.ControllerType.TallonSRX, steerId, Motor.Type.Falcon500);

    mDrive
        .addEncoder(new REVBoreEncoder())
        .inverted(true)
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs())
        .SimConfig(new SimConfig(true, 111.8, 10))
        .setName("mDrive" + name);
    mSteer
        .addEncoder(new REVBoreEncoder())
        .inverted(true)
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs())
        .SimConfig(new SimConfig(false, 111.8, 10))
        .setName("mSteer" + name);

    this.name = name;
  }

  public String getName() {
    return name;
  }

  public void setTargetState(SwerveModuleState targetState) {
    setTargetDriveVelocity(targetState.speedMetersPerSecond);
    setTargetSteerPosition(targetState.angle);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        mDrive.getPositoin(), Rotation2d.fromRadians(mSteer.getPositoin()));
  }

  private void setTargetSteerPosition(Rotation2d targetSteerPosition) {
    mSteer.setPositoin(targetSteerPosition.getRadians());

    Logger.recordOutput("SetTargetVelocity", targetSteerPosition.getRadians());
  }

  private void setTargetDriveVelocity(double targetDriveVelocity) {
    mDrive.setVelocity(targetDriveVelocity);
  }

  @Override
  public void disable() {
    mDrive.disable();
    mSteer.disable();
  }

  // -----------------------------
  @Override
  public void loadPreferences() {
  }

  @Override
  public void toLog(LogTable table) {
    mSteer.toLog(table);
    mDrive.toLog(table);
    table.put("234", 123);
  }

  @Override
  public void close() throws Exception {
    mSteer.close();
    mDrive.close();
  }
}
