package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

public class VehicleState {
  private static VehicleState instance;
  private static final InterpolatingDoubleTreeMap dynamicPivotMap =
      new InterpolatingDoubleTreeMap();

  static {
    dynamicPivotMap.put(18.4, 48. - 1);
    dynamicPivotMap.put(16.4, 48. - 1);
    dynamicPivotMap.put(10.4, 45. - 1);
    dynamicPivotMap.put(5.1, 37. - 1);
    dynamicPivotMap.put(0., 30. - 1);
    dynamicPivotMap.put(-2., 27. - 1);
    // dynamicPivotMap.put(-3.6, 25. - 1);
    dynamicPivotMap.put(-4.1, 25. - 2);
    dynamicPivotMap.put(-6.1, 23. - 3);
    dynamicPivotMap.put(-8.07, 21.25 - 2);
  }

  @Getter @Setter boolean shouldUpdateCenterTagAlignment = false;
  @Getter Rotation2d dynamicPivotAngle;
  @Getter Optional<Rotation2d> centerTagError = Optional.empty();

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

  public void updateCenterTagError(VisionInputs visionInputs) {
    if (visionInputs.tagId == 7 || visionInputs.tagId == 3) {
      centerTagError = Optional.of(Rotation2d.fromDegrees(visionInputs.horizontalOffsetFromTarget));
    } else {

      centerTagError = Optional.empty();
    }

    Logger.recordOutput("Center tag error has value", centerTagError.isPresent());
    if (centerTagError.isPresent()) {
      Logger.recordOutput("Center tag error", centerTagError.get());
    }
  }
}
