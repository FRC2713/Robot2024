package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  VisionInfo info;
  NetworkTable table;

  DoubleArraySubscriber botPose,
      botPoseWpiBlue,
      botPoseWpiRed,
      cameraPoseTargetSpace,
      targetPoseCameraSpace,
      targetPoseRobotSpace,
      botPoseTargetSpace,
      cameraPoseRobotSpace,
      tc;
  DoubleSubscriber aprilTagId, tv, tx, ty, ta, tl, cl, tshort, tlong, thor, tvert, getpipe, tclass;
  IntegerPublisher ledMode, cameraMode, stream, pipeline, snapshot;
  DoubleArrayPublisher crop, cameraPoseRobotSpacePub;

  public VisionIOLimelight(VisionInfo info) {
    this.info = info;
    table = NetworkTableInstance.getDefault().getTable(info.getNtTableName());

    // botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    // botPoseWpiBlue = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    // botPoseWpiRed = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
    // cameraPoseTargetSpace =
    //     table.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});
    // targetPoseCameraSpace =
    //     table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    // targetPoseRobotSpace =
    //     table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    // botPoseTargetSpace =
    //     table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});
    // cameraPoseRobotSpace =
    //     table.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[] {});
    // tc = table.getDoubleArrayTopic("tc").subscribe(new double[] {});

    // aprilTagId = table.getDoubleTopic("tid").subscribe(0);
    // tv = table.getDoubleTopic("tv").subscribe(0);
    // tx = table.getDoubleTopic("tx").subscribe(0);
    // ty = table.getDoubleTopic("ty").subscribe(0);
    // ta = table.getDoubleTopic("ta").subscribe(0);
    // tl = table.getDoubleTopic("tl").subscribe(0);
    // cl = table.getDoubleTopic("cl").subscribe(0);
    // tshort = table.getDoubleTopic("tshort").subscribe(0);
    // tlong = table.getDoubleTopic("tlong").subscribe(0);
    // thor = table.getDoubleTopic("thor").subscribe(0);
    // tvert = table.getDoubleTopic("tvert").subscribe(0);
    // getpipe = table.getDoubleTopic("getpipe").subscribe(0);
    // tclass = table.getDoubleTopic("tclass").subscribe(0);

    // json = table.getStringTopic("json").subscribe("{}");

    ledMode = table.getIntegerTopic("ledMode").publish();
    cameraMode = table.getIntegerTopic("camMode").publish();
    pipeline = table.getIntegerTopic("pipeline").publish();
    stream = table.getIntegerTopic("stream").publish();
    snapshot = table.getIntegerTopic("snapshot").publish();
    crop = table.getDoubleArrayTopic("crop").publish();
    cameraPoseRobotSpacePub = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
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
          Timer.getFPGATimestamp() - arr[6] / 1000.0);
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
    // var botPosePair = timestampedPoseFromLLArray(botPose.get());
    // var botPoseBluePair = timestampedPoseFromLLArray(botPoseWpiBlue.get());
    // inputs.botPoseBlue = botPoseBluePair.getFirst();

    // var botPoseRedPair = timestampedPoseFromLLArray(botPoseWpiRed.get());
    // var cameraPoseInTargetSpacePair = timestampedPoseFromLLArray(cameraPoseTargetSpace.get());
    // var targetPoseInCameraSpacePair = timestampedPoseFromLLArray(targetPoseCameraSpace.get());
    // var targetPoseInRobotSpacePair = timestampedPoseFromLLArray(targetPoseRobotSpace.get());
    // var botPoseInTargetSpacePair = timestampedPoseFromLLArray(botPoseTargetSpace.get());
    // var cameraPoseInRobotSpacePair = timestampedPoseFromLLArray(cameraPoseRobotSpace.get());

    // inputs.botPose = botPosePair.getFirst();
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

    // var jsonResults = LimelightHelpers.getLatestResults(getInfo().getNtTableName());
    // var botPoseBluePair =
    // timestampedPoseFromLLArray(jsonResults.targetingResults.botpose_wpiblue);
    // inputs.botPoseBlue = botPoseBluePair.getFirst();
    // inputs.botPoseBlueTimestamp = botPoseBluePair.getSecond();
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
    cameraMode.set(
        switch (mode) {
          case VISION -> 0;
          case DRIVER_CAM -> 1;
        });
  }

  @Override
  public void setPipeline(int pipeline) {
    this.pipeline.set(pipeline);
  }

  @Override
  public void setStreamMode(StreamMode streamMode) {
    stream.set(
        switch (streamMode) {
          case STANDARD -> 0;
          case PIP_MAIN -> 1;
          case PIP_SECONDARY -> 2;
        });
  }

  @Override
  public void setSnapshotMode(SnapshotMode mode) {
    snapshot.set(
        switch (mode) {
          case NONE -> 0;
          case TAKE_ONE -> 1;
        });
  }

  @Override
  public void setCrop(double x0, double x1, double y0, double y1) {
    crop.set(new double[] {x0, x1, y0, y1});
  }

  @Override
  public void setCameraPoseInRobotSpaceInternal(
      double forward, double side, double up, double roll, double pitch, double yaw) {
    cameraPoseRobotSpacePub.set(new double[] {forward, side, up, roll, pitch, yaw});
  }
}
