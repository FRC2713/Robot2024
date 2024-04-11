package frc.robot.subsystems.visionIO;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelightLib implements VisionIO {

  VisionInfo info;

  public VisionIOLimelightLib(VisionInfo info) {
    this.info = info;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    PoseEstimate x = null;
    if (Constants.LimeLightConstants.ENABLE_MEGATAG2) {
      x = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(this.info.getNtTableName());
    } else {
      x = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.info.getNtTableName());
    }
    inputs.botPoseBlue = x.pose;
    inputs.botPoseBlueTimestamp = x.timestampSeconds;
    inputs.hasTarget = x.tagCount > 0;
    inputs.tagCount = x.tagCount;
    inputs.totalLatencyMs = x.latency;
    inputs.averageTagArea = x.avgTagArea;
    inputs.averageTagDistanceFromCamera = x.avgTagDist;

    inputs.results = LimelightHelpers.getLatestResults(this.info.getNtTableName());
  }

  @Override
  public VisionInfo getInfo() {
    return info;
  }

  @Override
  public void setLEDMode(LEDMode mode) {

    switch (mode) {
      case FORCE_BLINK:
        LimelightHelpers.setLEDMode_ForceBlink(this.getInfo().getNtTableName());
        break;
      case FORCE_OFF:
        LimelightHelpers.setLEDMode_ForceOff(this.getInfo().getNtTableName());
        break;
      case FORCE_ON:
        LimelightHelpers.setLEDMode_ForceOn(this.getInfo().getNtTableName());
        break;
      case PIPELINE:
        LimelightHelpers.setLEDMode_PipelineControl(this.getInfo().getNtTableName());
        break;
      default:
        break;
    }
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
