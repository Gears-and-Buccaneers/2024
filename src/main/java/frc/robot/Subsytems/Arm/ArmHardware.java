package frc.robot.Subsytems.Arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.hardware.Motors.*;
import frc.lib.hardware.Motors.MotorControlers.MotorController;

public class ArmHardware implements ArmRequirments {
    // private final SmartMotor motorElvatorPivot;
    // private final SmartMotor motorElvatorExstend;
    // private final SmartMotor motorWristPivot;

    @AutoLogOutput
    private Mechanism2d mechSetpoint;
    private MechanismLigament2d elevatorSetpoint;
    private MechanismLigament2d wristSetpoint;

    @AutoLogOutput
    private Mechanism2d mechAcual;
    private MechanismLigament2d elevatorAcual;
    private MechanismLigament2d wristAcual;

    private String logName;

    public ArmHardware() {
        // motorElvatorPivot = new VP775(1);
        // motorElvatorExstend = new VP775(2);
        // motorWristPivot = new VP775(3);

        // configMotor(motorElvatorPivot);
        // configMotor(motorElvatorExstend);
        // configMotor(motorWristPivot);

        configMech();
    }

    public void setLogName(String logName) {
        this.logName = logName;
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

    public void perodic() {
        // elevatorAcual.setAngle(motorElvatorPivot.getRotation());
        // elevatorAcual.setLength(motorElvatorExstend.getRotation());
        // wristAcual.setAngle(motorWristPivot.getRotation());
    }

    public void stop() {

    }

    // -----------------------------
    @Override
    public void loadPreferences() {

    }

    public void setBrakeMode(boolean enable) {
        // motorElvatorPivot.brakeMode(enable);
        // motorElvatorExstend.brakeMode(enable);
        // motorWristPivot.brakeMode(enable);
    }

    @Override
    public void toLog(LogTable table) {
        //Logger.processInputs(logName + "/elvator Motor", motorElvatorExstend);
        table.put("234", 123);
    }

    @Override
    public void close() throws Exception {
        // motorElvatorPivot.close();
        // motorElvatorExstend.close();
        // motorWristPivot.close();
    }

    private void configMotor(MotorController motor) {
        // motor.inverted(true);
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

    @Override
    public void setSimpleName(String SimpleName) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSimpleName'");
    }
}
