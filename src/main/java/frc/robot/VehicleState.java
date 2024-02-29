package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import lombok.Getter;

public class VehicleState {
  private static VehicleState instance;

  @Getter Rotation2d dynamicPivotAngle;

  private VehicleState() {}

  public static VehicleState getInstance() {
    if (instance == null) {
      instance = new VehicleState();
    }

    return instance;
  }

  public void updateDynamicPivotAngle(VisionInputs visionInputs) {}
}
