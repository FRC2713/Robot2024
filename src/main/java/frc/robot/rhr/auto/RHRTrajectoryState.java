package frc.robot.rhr.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record RHRTrajectoryState(double timestamp, Pose2d pose, ChassisSpeeds chassisSpeeds) {
  public double getLinearSpeed() {
    return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }
}
