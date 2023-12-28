package frc.robot.Subsytems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsytems.SubsytemRequirments;

public interface ArmRequirments extends SubsytemRequirments {

    void wristAngleSetpoint(Rotation2d angle);

    void elevatorAngleSetpoint(Rotation2d angle);

    void elevatorLengthSetpoint(double ft);

    void perodic();

    void stop();
}
