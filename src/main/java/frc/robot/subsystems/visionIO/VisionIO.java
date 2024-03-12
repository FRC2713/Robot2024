package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionInputs {
    public Pose3d botPoseBlue = new Pose3d();
    // public Pose3d botPose = new Pose3d();
    // public Pose3d botPoseRed = new Pose3d();
    // public Pose3d cameraPoseInTargetSpace = new Pose3d();
    // public Pose3d targetPoseInCameraSpace = new Pose3d();
    // public Pose3d targetPoseInRobotSpace = new Pose3d();
    // public Pose3d botPoseInTargetSpace = new Pose3d();
    // public Pose3d cameraPoseInRobotSpace = new Pose3d();

    public double botPoseBlueTimestamp = 0;
    // public double botPoseTimestamp = 0;
    // public double botPoseRedTimestamp = 0;
    // public double cameraPoseInTargetSpaceTimestamp = 0;
    // public double targetPoseInCameraSpaceTimestamp = 0;
    // public double targetPoseInRobotSpaceTimestamp = 0;
    // public double botPoseInTargetSpaceTimestamp = 0;
    // public double cameraPoseInRobotSpaceTimestamp = 0;

    // public double aprilTagId = 0;
    // public boolean hasValidTarget = false;
    // public double horizontalOffsetFromTarget = 0;
    // public double verticalOffsetFromTarget = 0;
    // public double targetArea = 0;
    // public double pipelineLatency = 0;
    // public double captureLatency = 0;
    // public double totalLatency = 0;
    // public double shortestBoundingBoxSidelength = 0;
    // public double longestBoundingBoxSideLength = 0;
    // public double horizontalBoundingBoxSideLength = 0;
    // public double verticalBoundingBoxSideLength = 0;
    // public double activePipeline = 0;
    // public double neuralNetClassId = 0;
    // public double[] averageHsvColor = new double[] {};
    // public int targetCountFiducials = 0;

    public boolean hasTarget = false;
    public double horizontalOffsetFromTarget = 0;
    public double verticalOffsetFromTarget = 0;
    public double targetArea = 0;
    public double pipelineLatencyMs = 0;
    public double captureLatencyMs = 0;
    public double totalLatencyMs = 0;
    public double activePipeline = 0;
    public double tagCount = 0;
    public double tagSpan = 0;
    public double averageTagDistanceFromCamera = 0;
    public double averageTagArea = 0;
    public double tagId = 0;
  }

  public void updateInputs(VisionInputs inputs);

  public VisionInfo getInfo();

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

  public void setPriorityId(int tagId);
}
