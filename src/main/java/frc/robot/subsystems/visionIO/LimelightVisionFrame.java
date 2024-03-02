package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Builder;

@Builder
public class LimelightVisionFrame {
  double translationX;
  double translationY;
  double translationZ;
  double rotationRoll;
  double rotationPitch;
  double rotationYaw;
  double totalLatency;
  double tagCount;
  double tagSpan;
  double averageTagDistanceFromCamera;
  double averageTagArea;

  public Pose3d getPose() {
    return new Pose3d(
        new Translation3d(this.translationX, this.translationY, this.translationZ),
        new Rotation3d(this.rotationRoll, this.rotationPitch, this.rotationYaw));
  }
}
