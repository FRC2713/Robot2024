package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

public class VehicleState {
  private static VehicleState instance;
  private static final InterpolatingDoubleTreeMap dynamicPivotMap =
      new InterpolatingDoubleTreeMap();

  static {
    dynamicPivotMap.put(18.4, 48.);
    dynamicPivotMap.put(16.4, 48.);
    dynamicPivotMap.put(10.4, 45.);
    dynamicPivotMap.put(5.1, 37.);
    dynamicPivotMap.put(0., 30.);
    dynamicPivotMap.put(-2., 27.);
    dynamicPivotMap.put(-4.1, 25.);
    dynamicPivotMap.put(-6.1, 23.);
    dynamicPivotMap.put(-8.07, 21.25);
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
