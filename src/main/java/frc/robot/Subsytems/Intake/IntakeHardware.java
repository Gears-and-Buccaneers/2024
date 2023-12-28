package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
// import frc.lib.hardware.motorController.*;
import frc.lib.hardware.sensor.proximitySwitch.*;

public class IntakeHardware implements IntakeRequirments {
  private final CANSparkMax motor;
  private final ProximitySwitch switch1;

  private double setVolts = .1;

  private String logName;

  private static double kDt = 0.02;
  private static double kMaxVelocity = 1.75;
  private static double kMaxAcceleration = 0.75;
  private static double kP = 0.3;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
      kMaxAcceleration);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kG, kV);
  public RelativeEncoder encoder;

  private double encoderPos = 0;

  public IntakeHardware() {
    motor = new CANSparkMax(15, MotorType.kBrushed);
    motor.restoreFactoryDefaults();

    encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 2048);
    switch1 = new Huchoo(3);
    m_controller.setTolerance(10);
    // m_controller.

    initPrefrences();
    m_controller.enableContinuousInput(0, 1098);

    m_controller.setGoal(10);
    MathUtil.clamp(0, 0, 0);
  }

  public void setIntakeVoltage() {
    m_controller.setGoal(2);
    // motor.set(ControlMode.PercentOutput, setVolts);
  }

  public void setOutakeVoltage() {
    m_controller.setGoal(1);
    // motor.set(ControlMode.PercentOutput, -setVolts);
  }

  public void off() {
    motor.stopMotor();
    // motor.set(ControlMode.Disabled, 0);
  }

  public void periodic() {
    encoderPos = encoder.getVelocity() / 4;
    motor.setVoltage(
        m_controller.calculate(encoderPos));
  }

  public boolean isOpen() {
    return switch1.isOpen();
  }

  public boolean isClosed() {
    return switch1.isClosed();
  }

  // required things-------------------------------------------------

  @Override
  public void close() throws Exception {
    switch1.close();
  }

  private void initPrefrences() {
    Preferences.initDouble("maxVolts", setVolts);
  }

  public void loadPreferences() {
    setVolts = Preferences.getDouble("maxVolts", setVolts);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("SupplyCurrent", motor.getAppliedOutput());
    table.put("encoder Pos", encoderPos);
    table.put("encoder PositionConversionFactor", encoder.getPositionConversionFactor());
    table.put("encoder Velocity", encoder.getVelocity());
    // Logger.processInputs(logName + "/switch1", switch1);

  }

  @Override
  public void fromLog(LogTable table) {

  }

  public void setLogName(String logName) {
    this.logName = logName;
  }
}
