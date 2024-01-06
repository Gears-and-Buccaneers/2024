package frc.lib.hardware.Motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
//
import frc.lib.hardware.HardwareRequirements;
import frc.lib.hardware.Motors.MotorControllers.*;
import frc.lib.hardware.Motors.PID.*;
import frc.lib.hardware.sensor.encoders.Encoder;
//
import org.littletonrobotics.junction.LogTable;

public class Motor2 implements HardwareRequirements {
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
    TalonSRX(new Talon_SRX());

    MotorController mController;

    private ControllerType(MotorController mController) {
      this.mController = mController;
    }

    public MotorController config(int canID) {
      return mController.build(canID);
    }
  }

  // -----------------------------------------
  private final MotorController mController;
  private final Type mType;
  private final Encoder mEncoder;

  private boolean hasEncoder = false;

  private boolean isSimulated = false;

  private String name;

  public Motor2(ControllerType motorControllerType, int canID, Type motor) {
    this(motorControllerType, canID, motor, null);
    hasEncoder = false;
  }

  public Motor2(ControllerType motorControllerType, int canID, Type motor, Encoder encoder) {
    this.mController = motorControllerType.config(canID);
    this.mEncoder = encoder;
    this.mType = motor;
    this.name = "Motor" + canID;
    this.isSimulated = RobotBase.isSimulation();
    hasEncoder = true;
  }

  // ----------------- COnfigs ---------------------

  public Motor2 inverted(boolean enable) {
    mController.setInverted(enable);

    return this;
  }

  public Motor2 brakeMode(boolean enable) {
    mController.brakeMode(enable);

    return this;
  }

  private PIDController mFeedback;

  public Motor2 pidConfigs(PIDConfigs PIDConfigs) {
    mFeedback = new PIDController(1, 0, 0);

    return this;
  }

  public Motor2 EncoderConfigs(EncoderConfigs encoderConfigs) {

    return this;
  }

  public Motor2 setName(String name) {
    this.name = name;
    return this;
  }

  // controlling Motor contoler ---------------------------------
  public void setVolts(double Volts) {
    mController.runVolts(Volts);
  }

  public void setPositoin(double Positoin) {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    mFeedback.setSetpoint(Positoin);

    setVolts(clamp(mFeedback.calculate(getPositoin())));
  }

  public void setVelocity(double velocity) {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    mFeedback.setSetpoint(velocity);

    setVolts(clamp(mFeedback.calculate(getVelocity())));
  }

  public void disable() {
    mController.disable();
  }

  public double clamp(double setPoint) {
    return MathUtil.clamp(setPoint, -12.0, 12.0);
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
      return mEncoder.getPosition();
    } else {
      return 1;
    }
  }

  public double getVelocity() {
    if (!hasEncoder) {
      throw new Error("you dumb. This motor has no encoder");
    }
    if (RobotBase.isReal()) {
      return mEncoder.getVelocity();
    } else {
      return 1;
    }
  }

  // Required ---------------------------

  @Override
  public void toLog(LogTable table) {
    mController.toLog(table, name);
    table.put(name + "/Motor Type", mType.toString());
    if (hasEncoder) {
      table.put(name + "/Position (degrees)", getPositoin());
      table.put(name + "/Velocity (degreesPerSecond)", getVelocity());
      table.put(name + "/Velocity (MetersPerSecond)", getVelocity() * Units.inchesToMeters(2));
      // table.put(name + "/AppliedVolts (Volts)", mAppliedVolts);
      // table.put(name + "/CurrentDraw (Amps)", Math.abs(mSim.getCurrentDrawAmps()));
    }
  }

  @Override
  public boolean connected() {
    if (!mController.connected())
      return false;
    if (hasEncoder)
      return mEncoder.connected();
    return true;
  }
}
