package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class VehicleState {
  private static VehicleState instance;
  private static final InterpolatingDoubleTreeMap dynamicPivotMap =
      new InterpolatingDoubleTreeMap();

  static {
    dynamicPivotMap.put(0.0, 0.0);
  }

  @Getter Rotation2d dynamicPivotAngle;

  private VehicleState() {}

  public static VehicleState getInstance() {
    if (instance == null) {
      instance = new VehicleState();
    }

    return instance;
  }

  public void updateDynamicPivotAngle(double estimateDistanceToTag) {
    dynamicPivotAngle = Rotation2d.fromDegrees(dynamicPivotMap.get(estimateDistanceToTag));
  }
}
