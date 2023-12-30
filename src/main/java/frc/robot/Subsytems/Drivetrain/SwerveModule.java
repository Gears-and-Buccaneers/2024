package frc.robot.Subsytems.Drivetrain;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.hardware.Motors.Motor;
import frc.robot.Subsytems.SubsytemRequirments;

public class SwerveModule implements SubsytemRequirments {
  private final Motor mDrive;
  private final Motor mSteer;

  private final String name;

  public SwerveModule(String name) {
    mElvatorPivot = new Motor(Motor.ControllerType.TallonSRX, 1, Motor.Type.Falcon500);
    mElvatorExstend = new Motor(Motor.ControllerType.TallonSRX, 1, Motor.Type.Falcon500);
    this.name = name;
  }

  public void setTargetState(SwerveModuleState targetState) {
    setTargetDriveVelocity(targetState.speedMetersPerSecond);
    setTargetSteerPosition(targetState.angle);
  }

  /**
   * Getting the current swerve module position
   *
   * @return The drive speed and steer angle of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition();
  }

  private void setTargetSteerPosition(Rotation2d targetSteerPosition) {
  }

  private void setTargetDriveVelocity(double targetDriveVelocity) {
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void toLog(LogTable table) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'toLog'");
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'disable'");
  }
}
