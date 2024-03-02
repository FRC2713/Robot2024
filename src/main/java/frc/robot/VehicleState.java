package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class VehicleState {
  private static VehicleState instance;
  private static final InterpolatingDoubleTreeMap dynamicPivotMap =
      new InterpolatingDoubleTreeMap();

  static {
    dynamicPivotMap.put(21.45, 48.0);
    dynamicPivotMap.put(11.2, 45.0);
    dynamicPivotMap.put(5.2, 37.);
    dynamicPivotMap.put(0., 25.);
    dynamicPivotMap.put(-5., 20.);
    dynamicPivotMap.put(-8., 17.);
    dynamicPivotMap.put(-9.5, 16.125);
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
