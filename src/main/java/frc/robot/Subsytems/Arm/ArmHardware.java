package frc.robot.Subsytems.Arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.hardware.motorController.*;

public class ArmHardware implements ArmRequirments {
    private final SmartMotor motorElvatorPivot;
    private final SmartMotor motorElvatorExstend;
    private final SmartMotor motorWristPivot;

    @AutoLogOutput
    private Mechanism2d mechSetpoint;
    private MechanismLigament2d elevatorSetpoint;
    private MechanismLigament2d wristSetpoint;

    @AutoLogOutput
    private Mechanism2d mechAcual;
    private MechanismLigament2d elevatorAcual;
    private MechanismLigament2d wristAcual;

    public ArmHardware() {
        motorElvatorPivot = new Falcon500(1);
        motorElvatorExstend = new Falcon500(2);
        motorWristPivot = new Falcon500(3);

        configMotor(motorElvatorPivot);
        configMotor(motorElvatorExstend);
        configMotor(motorWristPivot);

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

    public void updateMech() {
        elevatorAcual.setAngle(motorElvatorPivot.getRotation());
        elevatorAcual.setLength(motorElvatorExstend.getRotation());
        wristAcual.setAngle(motorWristPivot.getRotation());
    }

    public void stop() {

    }

    // -----------------------------
    @Override
    public void loadPreferences() {

    }

    public void setBrakeMode(boolean enable) {
        motorElvatorPivot.brakeMode(enable);
        motorElvatorExstend.brakeMode(enable);
        motorWristPivot.brakeMode(enable);
    }

    @Override
    public void toLog(LogTable table) {
        // table.put("Motor1", motor1);
        // table.put("Motor2", motor2);
        // table.put("Motor3", motor3);
        // table.put("Motor4", motor4);
    }

    @Override
    public void close() throws Exception {
        motorElvatorPivot.close();
        motorElvatorExstend.close();
        motorWristPivot.close();
    }

    private void configMotor(Motor motor) {
        motor.inverted(true);
    }

    private void configMech() {
        // units are in inches
        mechSetpoint = new Mechanism2d(122, 126);
        // the mechanism root node
        MechanismRoot2d rootS = mechSetpoint.getRoot("arm", 50, 12);
        elevatorSetpoint = rootS.append(
                new MechanismLigament2d("elevator", 40, 90, 7, new Color8Bit(Color.kPurple)));
        wristSetpoint = elevatorSetpoint.append(
                new MechanismLigament2d("wrist", 15, 0, 5, new Color8Bit(Color.kPurple)));

        // units are in inches
        mechAcual = new Mechanism2d(122, 126);
        // the mechanism root node
        MechanismRoot2d rootA = mechAcual.getRoot("arm", 50, 12);
        elevatorAcual = rootA.append(
                new MechanismLigament2d("elevator", 40, 90, 8, new Color8Bit(Color.kCyan)));
        wristAcual = elevatorAcual.append(
                new MechanismLigament2d("wrist", 15, 0, 6, new Color8Bit(Color.kCyan)));
    }
}
