package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionIOLimelightJson implements VisionIO {

  VisionInfo info;

  public VisionIOLimelightJson(VisionInfo info) {
    this.info = info;
  }

  private Pair<Pose3d, Double> timestampedPoseFromLLArray(double[] arr) {
    if (arr.length == 0) {
      return new Pair<Pose3d, Double>(new Pose3d(), 0.0);
    }

    if (arr.length == 7) {
      return new Pair<Pose3d, Double>(
          new Pose3d(
              new Translation3d(arr[0], arr[1], arr[2]),
              new Rotation3d(
                  Units.degreesToRadians(arr[3]),
                  Units.degreesToRadians(arr[4]),
                  Units.degreesToRadians(arr[5]))),
          arr[6]);
    }

    return new Pair<Pose3d, Double>(
        new Pose3d(
            new Translation3d(arr[0], arr[1], arr[2]),
            new Rotation3d(
                Units.degreesToRadians(arr[3]),
                Units.degreesToRadians(arr[4]),
                Units.degreesToRadians(arr[5]))),
        0.0);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    var results = LimelightHelpers.getLatestResults(this.info.getNtTableName());
    inputs.targetCountFiducials = results.targetingResults.targets_Fiducials.length;

    // var botPosePair = timestampedPoseFromLLArray(botPose.get());
    // var botPoseBluePair = timestampedPoseFromLLArray(botPoseWpiBlue.get());
    // var botPoseRedPair = timestampedPoseFromLLArray(botPoseWpiRed.get());
    // var cameraPoseInTargetSpacePair = timestampedPoseFromLLArray(cameraPoseTargetSpace.get());
    // var targetPoseInCameraSpacePair = timestampedPoseFromLLArray(targetPoseCameraSpace.get());
    // var targetPoseInRobotSpacePair = timestampedPoseFromLLArray(targetPoseRobotSpace.get());
    // var botPoseInTargetSpacePair = timestampedPoseFromLLArray(botPoseTargetSpace.get());
    // var cameraPoseInRobotSpacePair = timestampedPoseFromLLArray(cameraPoseRobotSpace.get());

    // inputs.botPose = botPosePair.getFirst();
    // inputs.botPoseBlue = botPoseBluePair.getFirst();
    // inputs.botPoseRed = botPoseRedPair.getFirst();
    // inputs.cameraPoseInTargetSpace = cameraPoseInTargetSpacePair.getFirst();
    // inputs.targetPoseInCameraSpace = targetPoseInCameraSpacePair.getFirst();
    // inputs.targetPoseInRobotSpace = targetPoseInRobotSpacePair.getFirst();
    // inputs.botPoseInTargetSpace = botPoseInTargetSpacePair.getFirst();
    // inputs.cameraPoseInRobotSpace = cameraPoseInRobotSpacePair.getFirst();
    // inputs.botPoseTimestamp = botPosePair.getSecond();
    // inputs.botPoseBlueTimestamp = botPoseBluePair.getSecond();
    // inputs.botPoseRedTimestamp = botPoseRedPair.getSecond();
    // inputs.cameraPoseInTargetSpaceTimestamp = cameraPoseInTargetSpacePair.getSecond();
    // inputs.targetPoseInCameraSpaceTimestamp = targetPoseInCameraSpacePair.getSecond();
    // inputs.targetPoseInRobotSpaceTimestamp = targetPoseInRobotSpacePair.getSecond();
    // inputs.botPoseInTargetSpaceTimestamp = botPoseInTargetSpacePair.getSecond();
    // inputs.cameraPoseInRobotSpaceTimestamp = cameraPoseInRobotSpacePair.getSecond();

    // inputs.averageHsvColor = tc.get();
    // inputs.aprilTagId = aprilTagId.get();
    // inputs.hasValidTarget = tv.get() == 1;
    // inputs.horizontalOffsetFromTarget = tx.get();
    // inputs.verticalOffsetFromTarget = ty.get();
    // inputs.targetArea = ta.get();
    // inputs.pipelineLatency = tl.get();
    // inputs.captureLatency = cl.get();
    // inputs.totalLatency = inputs.pipelineLatency + inputs.captureLatency;
    // inputs.shortestBoundingBoxSidelength = tshort.get();
    // inputs.longestBoundingBoxSideLength = tlong.get();
    // inputs.horizontalBoundingBoxSideLength = thor.get();
    // inputs.verticalBoundingBoxSideLength = tvert.get();
    // inputs.activePipeline = getpipe.get();
    // inputs.neuralNetClassId = tclass.get();

  }

  @Override
  public VisionInfo getInfo() {
    return info;
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    // ledMode.set(
    //     switch (mode) {
    //       case PIPELINE -> 0;
    //       case FORCE_OFF -> 1;
    //       case FORCE_BLINK -> 2;
    //       case FORCE_ON -> 3;
    //     });
  }

  @Override
  public void setCameraMode(CameraMode mode) {
    // cameraMode.set(
    //     switch (mode) {
    //       case VISION -> 0;
    //       case DRIVER_CAM -> 1;
    //     });
  }

  @Override
  public void setPipeline(int pipeline) {
    // this.pipeline.set(pipeline);
  }

  @Override
  public void setStreamMode(StreamMode streamMode) {
    // stream.set(
    //     switch (streamMode) {
    //       case STANDARD -> 0;
    //       case PIP_MAIN -> 1;
    //       case PIP_SECONDARY -> 2;
    //     });
  }

  @Override
  public void setSnapshotMode(SnapshotMode mode) {
    // snapshot.set(
    //     switch (mode) {
    //       case NONE -> 0;
    //       case TAKE_ONE -> 1;
    //     });
  }

  @Override
  public void setCrop(double x0, double x1, double y0, double y1) {
    // crop.set(new double[] {x0, x1, y0, y1});
  }

  @Override
  public void setCameraPoseInRobotSpaceInternal(
      double forward, double side, double up, double roll, double pitch, double yaw) {
    // cameraPoseRobotSpacePub.set(new double[] {forward, side, up, roll, pitch, yaw});
  }
}
