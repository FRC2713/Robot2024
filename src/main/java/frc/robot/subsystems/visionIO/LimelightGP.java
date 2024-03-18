package frc.robot.subsystems.visionIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ObjectDetection;
import java.util.ArrayList;

public class LimelightGP extends SubsystemBase {
  private VisionInfo info;
  public ArrayList<ObjectDetection> detections = new ArrayList<>();

  public LimelightGP(VisionInfo info) {
    this.info = info;
  }

  @Override
  public void periodic() {
    var table = LimelightHelpers.getLatestResults(info.getNtTableName());
    detections.clear();
    for (var detection : table.targetingResults.targets_Detector) {
      detections.add(
          ObjectDetection.fromImageRelativeCentre(
              320,
              240,
              detection.tx_pixels,
              detection.ty_pixels,
              Math.sqrt(detection.ta),
              Math.sqrt(detection.ta)));
    }
  }
}
