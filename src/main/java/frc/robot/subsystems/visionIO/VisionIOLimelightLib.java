package frc.robot.subsystems.visionIO;

import frc.robot.util.LimelightHelpers;

public class VisionIOLimelightLib implements VisionIO {

  VisionInfo info;

  public VisionIOLimelightLib(VisionInfo info) {
    this.info = info;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    var x = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.info.getNtTableName());
    inputs.botPoseBlue = x.pose;
    inputs.botPoseBlueTimestamp = x.timestampSeconds;
    inputs.hasTarget = x.tagCount > 0;
    inputs.tagCount = x.tagCount;
    inputs.totalLatencyMs = x.latency;
  }

  @Override
  public VisionInfo getInfo() {
    return info;
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setLEDMode'");
  }

  @Override
  public void setCameraMode(CameraMode mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCameraMode'");
  }

  @Override
  public void setPipeline(int pipeline) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPipeline'");
  }

  @Override
  public void setStreamMode(StreamMode streamMode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setStreamMode'");
  }

  @Override
  public void setSnapshotMode(SnapshotMode mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setSnapshotMode'");
  }

  @Override
  public void setCrop(double x0, double x1, double y0, double y1) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCrop'");
  }

  @Override
  public void setCameraPoseInRobotSpaceInternal(
      double forward, double side, double up, double roll, double pitch, double yaw) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'setCameraPoseInRobotSpaceInternal'");
  }

  @Override
  public void setPriorityId(int tagId) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPriorityId'");
  }
}
