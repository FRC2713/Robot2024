package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import lombok.Getter;

public class RobotState {
    private static RobotState instance;

    @Getter Rotation2d dynamicPivotAngle;

    private RobotState() {}

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }

        return instance;
    }

    public void updateDynamicPivotAngle(VisionInputs visionInputs) {}
}