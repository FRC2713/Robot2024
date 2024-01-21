package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionInputs {
    double[] botPose = new double[] {};
    double[] botPoseBlue = new double[] {};
    double[] botPoseRed = new double[] {};
    double[] cameraPoseInTargetSpace = new double[] {};
    double[] targetPoseInCameraSpace = new double[] {};
    double[] targetPoseInRobotSpace = new double[] {};
    double[] botPoseInTargetSpace = new double[] {};
    double[] cameraPoseInRobotSpace = new double[] {};
    double aprilTagId = 0;

    boolean hasValidTarget = false;
    double horizontalOffsetFromTarget = 0;
    double verticalOffsetFromTarget = 0;
    double targetArea = 0;
    double pipelineLatency = 0;
    double captureLatency = 0;
    double totalLatency = 0;
    double shortestBoundingBoxSidelength = 0;
    double longestBoundingBoxSideLength = 0;
    double horizontalBoundingBoxSideLength = 0;
    double verticalBoundingBoxSideLength = 0;
    double activePipeline = 0;
    double neuralNetClassId = 0;
    double[] averageHsvColor = new double[] {};
  }

  public void updateInputs(VisionInputs inputs);

  public String getName();

  public enum LEDMode {
    PIPELINE,
    FORCE_OFF,
    FORCE_BLINK,
    FORCE_ON
  };

  public enum CameraMode {
    VISION,
    DRIVER_CAM
  };

  public enum StreamMode {
    STANDARD,
    PIP_MAIN,
    PIP_SECONDARY
  };

  public enum SnapshotMode {
    NONE,
    TAKE_ONE
  };

  public void setLEDMode(LEDMode mode);

  public void setCameraMode(CameraMode mode);

  public void setPipeline(int pipeline);

  public void setStreamMode(StreamMode streamMode);

  public void setSnapshotMode(SnapshotMode mode);

  public void setCrop(double x0, double x1, double y0, double y1);

  public void setCameraPoseInRobotSpaceInternal(
      double forward, double side, double up, double roll, double pitch, double yaw);

  public default void setCameraPoseInRobotSpace(Pose3d pose) {
    setCameraPoseInRobotSpaceInternal(
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }
}
