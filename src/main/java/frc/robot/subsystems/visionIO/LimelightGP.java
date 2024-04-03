package frc.robot.subsystems.visionIO;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;

public class LimelightGP extends SubsystemBase {
  private VisionInfo info;
  public LimelightTarget_Detector[] detections = new LimelightTarget_Detector[] {};
  private boolean isSimulation;

  public LimelightGP(VisionInfo info, boolean isSimulation) {
    this.info = info;
    this.isSimulation = isSimulation;
  }

  @Override
  public void periodic() {
    if (!isSimulation) {
      var table = LimelightHelpers.getLatestResults(info.getNtTableName());
      detections = table.targetingResults.targets_Detector;
      Logger.recordOutput("LimelightGP/Num Detection", detections.length);
    } else {
      var detection = new LimelightTarget_Detector();
      detection.tx = 30;
      detection.ty = -5;
      detection.ta = 0.001;
      detections = new LimelightTarget_Detector[] {detection};
    }
  }
  // Logger.recordOutput("Limelight/tx", detections[0].tx);
}
