package frc.robot.subsystems.visionIO;

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
}
