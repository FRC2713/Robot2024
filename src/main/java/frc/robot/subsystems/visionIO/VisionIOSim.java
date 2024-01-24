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
  String name;

  AprilTagFieldLayout layout;

  @SneakyThrows
  public VisionIOSim(String name) {
    this.name = name;
    visionSim = new VisionSystemSim(name);
    layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    visionSim.addAprilTags(layout);

    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(Math.hypot(63.3, 49.7)));
    cameraProperties.setCalibError(.25, .08);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(35);
    cameraProperties.setLatencyStdDevMs(5);

    cameraHw = new PhotonCamera(name);
    cameraSim = new PhotonCameraSim(cameraHw, cameraProperties);
    visionSim.addCamera(cameraSim, new Transform3d());
    SmartDashboard.putData(visionSim.getDebugField());
  }

  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    visionSim.update(Robot.swerveDrive.getUsablePose());

    var res = cameraHw.getLatestResult();

    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var target = res.getBestTarget();
      var robotPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(),
              layout.getTagPose(target.getFiducialId()).get(),
              new Transform3d());

      inputs.botPose = robotPose;
      inputs.botPoseTimestamp = imageCaptureTime;
    } else {
      inputs.botPose = new Pose3d();
      inputs.botPoseTimestamp = 0.0;
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
}
