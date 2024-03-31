package frc;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {
    }

    // Don't mess with this
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}