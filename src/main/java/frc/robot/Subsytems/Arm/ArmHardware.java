package frc.robot.Subsytems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.hardware.Motors.*;
import frc.lib.hardware.Motors.PID.EncoderConfigs;
import frc.lib.hardware.Motors.PID.PIDConfigs;
import frc.lib.hardware.sensor.encoders.REVBoreEncoder;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;

public class ArmHardware implements ArmRequirments {
  private final Motor mElvatorPivot;
  private final Motor mElvatorExstend;
  private final Motor mWristPivot;

  @AutoLogOutput private Mechanism2d mechSetpoint;
  private MechanismLigament2d elevatorSetpoint;
  private MechanismLigament2d wristSetpoint;

  @AutoLogOutput private Mechanism2d mechAcual;
  private MechanismLigament2d elevatorAcual;
  private MechanismLigament2d wristAcual;

  public ArmHardware() {
    mElvatorPivot = new Motor(Motor.ControllerType.TallonSRX, 1, Motor.Type.Falcon500);
    mElvatorExstend = new Motor(Motor.ControllerType.TallonSRX, 1, Motor.Type.Falcon500);
    mWristPivot = new Motor(Motor.ControllerType.TallonSRX, 1, Motor.Type.Falcon500);

    mElvatorPivot
        .addEncoder(new REVBoreEncoder())
        .inverted(true)
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs())
        .setName("mElvatorPivot");

    mElvatorExstend
        .addEncoder(new REVBoreEncoder())
        .inverted(true)
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs())
        .setName("mElvatorExstend");

    mWristPivot
        .addEncoder(new REVBoreEncoder())
        .inverted(true)
        .pidConfigs(new PIDConfigs())
        .EncoderConfigs(new EncoderConfigs())
        .setName("mWristPivot");

    configMech();
  }

  @Override
  public void elevatorAngleSetpoint(Rotation2d angle) {
    elevatorSetpoint.setAngle(angle);
  }

  @Override
  public void wristAngleSetpoint(Rotation2d angle) {
    wristSetpoint.setAngle(angle);
  }

  @Override
  public void elevatorLengthSetpoint(double ft) {
    elevatorSetpoint.setLength(ft);
  }

  @Override
  public void periodic() {
    mElvatorPivot.setPositoin(elevatorSetpoint.getAngle());
    mElvatorExstend.setPositoin(elevatorSetpoint.getLength());
    mWristPivot.setPositoin(wristSetpoint.getAngle());

    elevatorAcual.setAngle(mElvatorPivot.getPositoin());
    elevatorAcual.setLength(mElvatorExstend.getPositoin());
    wristAcual.setAngle(mWristPivot.getPositoin());
  }

  public void setBrakeMode(boolean enable) {
    mElvatorPivot.brakeMode(enable);
    mElvatorExstend.brakeMode(enable);
    mWristPivot.brakeMode(enable);
  }

  @Override
  public void disable() {
    mElvatorPivot.disable();
    mElvatorExstend.disable();
    mWristPivot.disable();
  }

  // -----------------------------
  @Override
  public void loadPreferences() {}

  @Override
  public void toLog(LogTable table) {
    mElvatorPivot.toLog(table);
    mElvatorExstend.toLog(table);
    mWristPivot.toLog(table);
    table.put("234", 123);
  }

  @Override
  public void close() throws Exception {
    mElvatorPivot.close();
    mElvatorExstend.close();
    mWristPivot.close();
  }

  private void configMech() {
    // units are in inches
    mechSetpoint = new Mechanism2d(122, 126);
    // the mechanism root node
    MechanismRoot2d rootS = mechSetpoint.getRoot("arm", 50, 12);
    elevatorSetpoint =
        rootS.append(new MechanismLigament2d("elevator", 40, 90, 7, new Color8Bit(Color.kPurple)));
    wristSetpoint =
        elevatorSetpoint.append(
            new MechanismLigament2d("wrist", 15, 0, 5, new Color8Bit(Color.kPurple)));

    // units are in inches
    mechAcual = new Mechanism2d(122, 126);
    // the mechanism root node
    MechanismRoot2d rootA = mechAcual.getRoot("arm", 50, 12);
    elevatorAcual =
        rootA.append(new MechanismLigament2d("elevator", 40, 90, 8, new Color8Bit(Color.kCyan)));
    wristAcual =
        elevatorAcual.append(
            new MechanismLigament2d("wrist", 15, 0, 6, new Color8Bit(Color.kCyan)));
  }
}
