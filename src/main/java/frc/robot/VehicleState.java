package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.util.SwerveHeadingController;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class VehicleState {
  private static VehicleState instance;
  private static final InterpolatingDoubleTreeMap dynamicPivotMap =
      new InterpolatingDoubleTreeMap();

  static {
    // dynamicPivotMap.put(18.4, 48. - 1);
    // dynamicPivotMap.put(16.4, 48. - 1);
    // dynamicPivotMap.put(10.4, 45. - 1);
    // dynamicPivotMap.put(5.1, 37. - 1);
    // dynamicPivotMap.put(0., 30. - 1);
    // dynamicPivotMap.put(-2., 27. - 1);
    // // dynamicPivotMap.put(-3.6, 25. - 1);
    // dynamicPivotMap.put(-4.1, 25. - 2);
    // dynamicPivotMap.put(-6.1, 23. - 3);
    // dynamicPivotMap.put(-8.07, 21.25 - 2);

    dynamicPivotMap.put(1.442698092790545, 47.);
    dynamicPivotMap.put(1.67, 47.0);
    dynamicPivotMap.put(2.08, 37.0);
    dynamicPivotMap.put(2.31, 35.0);
    dynamicPivotMap.put(2.61, 31.0);
    dynamicPivotMap.put(3.03, 28.5);
    dynamicPivotMap.put(3.55, 25.);
    dynamicPivotMap.put(4.01, 24.5);
  }

  @Getter @Setter boolean shouldUpdateCenterTagAlignment = false;
  @Getter Rotation2d dynamicPivotAngle = Rotation2d.fromDegrees(45);
  @Getter Optional<Rotation2d> centerTagError = Optional.empty();

  private VehicleState() {}

  public static VehicleState getInstance() {
    if (instance == null) {
      instance = new VehicleState();
    }

    return instance;
  }

  public void updateDynamicPivotAngle(
      VisionInputs leftVisionInputs, VisionInputs rightVisionInputs) {
    // dynamicPivotAngle = Rotation2d.fromDegrees(dynamicPivotMap.get(estimateDistanceToTag));
    // Logger.recordOutput("Dynamic pivot angle", dynamicPivotAngle);

    boolean seeSpeakerLeft = false, seeSpeakerRight = false;

    var leftFiducials = leftVisionInputs.results.targetingResults.targets_Fiducials;
    var rightFiducials = rightVisionInputs.results.targetingResults.targets_Fiducials;
    for (int i = 0; i < leftFiducials.length; i++) {
      if (leftFiducials[i].fiducialID == 3
          || leftFiducials[i].fiducialID == 4
          || leftFiducials[i].fiducialID == 7
          || leftFiducials[i].fiducialID == 8) {
        seeSpeakerLeft = true;
        break;
      }
    }

    for (int i = 0; i < rightFiducials.length; i++) {
      if (rightFiducials[i].fiducialID == 3
          || rightFiducials[i].fiducialID == 4
          || rightFiducials[i].fiducialID == 7
          || rightFiducials[i].fiducialID == 8) {
        seeSpeakerRight = true;
        break;
      }
    }

    Logger.recordOutput("OTF/See Speaker Left", seeSpeakerLeft);
    Logger.recordOutput("OTF/See Speaker Right", seeSpeakerRight);

    if (seeSpeakerLeft && seeSpeakerRight) {
      var avgAvgDist =
          (leftVisionInputs.averageTagDistanceFromCamera
                  + rightVisionInputs.averageTagDistanceFromCamera)
              / 2.0;
      Logger.recordOutput("OTF/Avg Avg Dist", avgAvgDist);
      dynamicPivotAngle = Rotation2d.fromDegrees(dynamicPivotMap.get(avgAvgDist));
      Logger.recordOutput("OTF/Dynamic pivot angle", dynamicPivotAngle);
    }
  }

  public void updateCenterTagError(VisionInputs leftVisionInputs, VisionInputs rightVisionInputs) {
    // if ((leftVisionInputs.tagId == 7 || leftVisionInputs.tagId == 3)
    //     && (rightVisionInputs.tagId == 7 || rightVisionInputs.tagId == 3)) {
    //   centerTagError =
    //       Optional.of(
    //           Rotation2d.fromDegrees(
    //               leftVisionInputs.horizontalOffsetFromTarget
    //                   + rightVisionInputs.horizontalOffsetFromTarget));
    //   Logger.recordOutput("Center tag error", centerTagError.get());
    // } else {
    //   centerTagError = Optional.empty();
    // }

    Logger.recordOutput("Center tag error has value", centerTagError.isPresent());
    if (centerTagError.isPresent()) {
      Logger.recordOutput("Center tag error", centerTagError.get());
    }
  }

  public boolean hasGPLock = false;
  public LimelightTarget_Detector closestResult = new LimelightTarget_Detector();
  public Rotation2d GPyaw = new Rotation2d();

  public void resetClosestGP() {
    hasGPLock = false;
    closestResult = new LimelightTarget_Detector();
    closestResult.tx = 0;
    closestResult.ty = 0;
    closestResult.ta = 0;
  }

  public ChassisSpeeds goClosestGP() {
    Logger.recordOutput("OTF/DrivingToGP/Doing it", true);
    Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Everything good");

    double angle = (-1 * closestResult.tx);

    Logger.recordOutput("OTF/DrivingToGP/Angle", angle);

    SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(angle).plus(GPyaw));

    double xSpeed =
        (Math.abs(
                    MathUtil.applyDeadband(
                        -Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE))
                + Math.abs(
                    MathUtil.applyDeadband(
                        -Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)))
            * 2;

    return ChassisSpeeds.fromRobotRelativeSpeeds(
        xSpeed,
        0,
        Units.degreesToRadians(SwerveHeadingController.getInstance().update()),
        Rotation2d.fromDegrees(0));
  }
}
