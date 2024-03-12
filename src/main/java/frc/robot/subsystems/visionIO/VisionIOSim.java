package frc.robot.subsystems.visionIO;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.SneakyThrows;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

  VisionSystemSim visionSim;
  PhotonCamera cameraHw;
  PhotonCameraSim cameraSim;

  AprilTagFieldLayout layout;
  VisionInfo info;

  @SneakyThrows
  public VisionIOSim(VisionInfo info) {
    this.info = info;
    visionSim = new VisionSystemSim(info.getNtTableName());
    layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    visionSim.addAprilTags(layout);

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    cameraHw = new PhotonCamera(info.getNtTableName());
    cameraSim = new PhotonCameraSim(cameraHw, cameraProp);
    cameraSim.enableDrawWireframe(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableRawStream(true);
    visionSim.addCamera(cameraSim, new Transform3d());
    SmartDashboard.putData(visionSim.getDebugField());
  }

  public VisionInfo getName() {
    return info;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    visionSim.update(Robot.swerveDrive.getUsablePose());
    visionSim
        .getDebugField()
        .getObject("VisionEstimation")
        .setPose(Robot.swerveDrive.getUsablePose());

    var res = cameraHw.getLatestResult();

    inputs.hasTarget = res.hasTargets();

    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var target = res.getBestTarget();
      var robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(),
              layout.getTagPose(target.getFiducialId()).get(),
              new Transform3d());

      inputs.botPoseBlue = robotPose;
      inputs.botPoseBlueTimestamp = imageCaptureTime;
      inputs.horizontalOffsetFromTarget = target.getYaw();
      inputs.verticalOffsetFromTarget = target.getPitch();
      inputs.targetArea = target.getArea();
      inputs.pipelineLatencyMs = res.getLatencyMillis();
      inputs.captureLatencyMs = 0.0;
      inputs.activePipeline = 0;
      inputs.tagCount = res.getTargets().size();
      inputs.tagId = res.getBestTarget().getFiducialId();
    } else {
      inputs.botPoseBlue = new Pose3d();
      inputs.botPoseBlueTimestamp = 0.0;
      inputs.horizontalOffsetFromTarget = 0.0;
      inputs.verticalOffsetFromTarget = 0.0;
      inputs.targetArea = 0.0;
      inputs.pipelineLatencyMs = 0.0;
      inputs.captureLatencyMs = 0.0;
      inputs.activePipeline = 0;
      inputs.tagCount = 0;
    }
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
  public VisionInfo getInfo() {
    return info;
  }

  @Override
  public void setPriorityId(int tagId) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPriorityId'");
  }
}
