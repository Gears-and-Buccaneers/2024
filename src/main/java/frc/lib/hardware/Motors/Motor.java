package frc.lib.hardware.Motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.hardware.HardwareRequirments;
import frc.lib.hardware.Motors.MotorControlers.MotorController;
import frc.lib.hardware.Motors.MotorControlers.Talon_SRX;
import frc.lib.hardware.Motors.PID.EncoderConfigs;
import frc.lib.hardware.Motors.PID.PIDConfigs;
import frc.lib.hardware.Motors.PID.SimConfig;
import frc.lib.hardware.sensor.encoders.Encoder;
import frc.robot.Robot;
import org.littletonrobotics.junction.LogTable;

public class Motor implements HardwareRequirments {
  // Enums --------------
  public enum Type {
    CIM(DCMotor.getFalcon500(1)),
    Falcon500(DCMotor.getFalcon500Foc(1)),
    VP775(DCMotor.getFalcon500(1));

    private DCMotor dcMotor;

    private Type(DCMotor dcMotor) {
      this.dcMotor = dcMotor;
    }

    protected DCMotor config() {
      return this.dcMotor;
    }
  }

  public enum ControllerType {
    TallonSRX(new Talon_SRX());

    MotorController mController;

    private ControllerType(MotorController mController) {
      this.mController = mController;
    }

    public MotorController config(int canID) {
      return mController.config(canID);
    }
  }

  // -----------------------------------------
  private final MotorController mController;
  private final Type mType;
  private Encoder mEncoder;

  private boolean hasEncoder = false;
  private boolean isSimulated = false;

  private String name;

  public Motor(ControllerType motorControllerType, int canID, Type motor) {
    this.mController = motorControllerType.config(canID);
    this.mType = motor;
    this.name = "Motor" + canID;
  }

  // ----------------- COnfigs ---------------------
  public Motor addEncoder(Encoder encoder) {
    this.mEncoder = encoder;
    hasEncoder = true;
    return this;
  }

  public Motor inverted(boolean enable) {
    mController.setInverted(enable);

    return this;
  }

  public Motor brakeMode(boolean enable) {
    mController.brakeMode(enable);

    return this;
  }

  private PIDController mFeedback;

  public Motor pidConfigs(PIDConfigs PIDConfigs) {
    mFeedback = new PIDController(1, 0, 0);

    return this;
  }

  public Motor EncoderConfigs(EncoderConfigs encoderConfigs) {

    return this;
  }

  private PIDController mSimFeedback;
  private FlywheelSim mSim;
  private double mSimPositionDeg;
  private double mSimPositionMeters;
  private double mSimVelocityDeg;
  private double mSimVelocityMeters;
  private boolean simVelocity = false;

  private double mAppliedVolts;

  public Motor SimConfig(SimConfig simConfig) {
    isSimulated = true;
    mSim = new FlywheelSim(mType.config(), 6.75, 0.0025);
    mSimFeedback = new PIDController(1, 0, 0);

    simVelocity = simConfig.simVelocity();
    return this;
  }

  public Motor setName(String name) {
    this.name = name;
    return this;
  }

  // set ---------------------------------
  public void setVolts(double Volts) {
    mController.runVolts(Volts);
  }

  public void setPositoin(double Positoin) {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    mSimFeedback.setSetpoint(Positoin);
    mFeedback.setSetpoint(Positoin);

    mAppliedVolts = mFeedback.calculate(getPositoin());
    mAppliedVolts = MathUtil.clamp(mAppliedVolts, -12.0, 12.0);

    setVolts(mAppliedVolts);
  }

  public void setVelocity(double velocity) {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    mSimFeedback.setSetpoint(velocity);
    mFeedback.setSetpoint(velocity);

    mAppliedVolts = mFeedback.calculate(getVelocity());
    mAppliedVolts = MathUtil.clamp(mAppliedVolts, -12.0, 12.0);

    setVolts(mAppliedVolts);
  }

  public void disable() {
    mController.disable();
  }

  // get ---------------------------------
  public int getCanID() {
    return mController.getCanID();
  }

  @Override
  public void close() throws Exception {
    mController.close();
    mEncoder.close();
  }

  public double getPositoin() {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    if (RobotBase.isReal()) {
      return mEncoder.getPositoin();
    } else {
      return mSimPositionDeg;
    }
  }

  public double getVelocity() {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    if (RobotBase.isReal()) {
      return mEncoder.getVelocity();
    } else {
      return mSimPositionDeg;
    }
  }

  // Required ---------------------------

  @Override
  public void toLog(LogTable table) {
    mController.toLog(table, name);
    table.put(name + "/Motor Type", mType.toString());
    if (hasEncoder) {
      table.put(name + "/Position (degrees)", getPositoin());
      table.put(name + "/Velocity (degrees/Second)", getVelocity());
      table.put(name + "/Velocity (Meters/Second)", getVelocity() * Units.inchesToMeters(2));
      table.put(name + "/AppliedVolts (Volts)", mAppliedVolts);
      table.put(name + "/CurrentDraw (Amps)", Math.abs(mSim.getCurrentDrawAmps()));
      if (isSimulated) {
        logSim(table);
      }
    }
  }

  private void logSim(LogTable table) {
    double AppliedVolts;
    if (simVelocity) {
      AppliedVolts = mSimFeedback.calculate(mSimVelocityDeg);
    } else {
      AppliedVolts = mSimFeedback.calculate(mSimPositionDeg);
    }
    // mSimFeedback.

    AppliedVolts = MathUtil.clamp(AppliedVolts, -12.0, 12.0);

    mSim.setInputVoltage(AppliedVolts);

    mSim.update(Robot.defaultPeriodSecs);

    mSimPositionDeg =
        mSimPositionDeg + (mSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs);
    mSimPositionMeters +=
        (mSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs) * Units.inchesToMeters(2);
    mSimVelocityDeg = mSim.getAngularVelocityRadPerSec();
    mSimPositionMeters = mSim.getAngularVelocityRadPerSec() * Units.inchesToMeters(2);
    // --------------------------------------------------------
    table.put(name + "/SimPos/Position (degrees)", mSimPositionDeg);
    table.put(name + "/SimPos/Position (Meters)", mSimPositionMeters);
    table.put(name + "/SimPos/Velocity (degrees/Second)", mSimVelocityDeg);
    table.put(name + "/SimPos/Velocity (Meters/Second)", mSimVelocityMeters);
    table.put(name + "/SimPos/AppliedVolts (Volts)", AppliedVolts);
    table.put(name + "/SimPos/CurrentDraw (Amps)", Math.abs(mSim.getCurrentDrawAmps()));
    if (simVelocity) {
      table.put(name + "/SimPos/Targot Velocity (degrees)", mSimFeedback.getSetpoint());
    } else {
      table.put(name + "/SimPos/Targot Position (degrees)", mSimFeedback.getSetpoint());
    }
  }

  @Override
  public boolean connected() {
    if (!mController.connected()) return false;
    if (hasEncoder) return mEncoder.connected();
    return true;
  }
}
