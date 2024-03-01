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
  IntegerPublisher ledMode, cameraMode, stream, pipeline, snapshot, priorityId;
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

    aprilTagId = table.getDoubleTopic("tid").subscribe(0);
    tv = table.getDoubleTopic("tv").subscribe(0);
    tx = table.getDoubleTopic("tx").subscribe(0);
    ty = table.getDoubleTopic("ty").subscribe(0);
    ta = table.getDoubleTopic("ta").subscribe(0);
    tl = table.getDoubleTopic("tl").subscribe(0);
    cl = table.getDoubleTopic("cl").subscribe(0);
    tshort = table.getDoubleTopic("tshort").subscribe(0);
    tlong = table.getDoubleTopic("tlong").subscribe(0);
    thor = table.getDoubleTopic("thor").subscribe(0);
    tvert = table.getDoubleTopic("tvert").subscribe(0);
    getpipe = table.getDoubleTopic("getpipe").subscribe(0);
    tclass = table.getDoubleTopic("tclass").subscribe(0);

    ledMode = table.getIntegerTopic("ledMode").publish();
    cameraMode = table.getIntegerTopic("camMode").publish();
    pipeline = table.getIntegerTopic("pipeline").publish();
    stream = table.getIntegerTopic("stream").publish();
    snapshot = table.getIntegerTopic("snapshot").publish();
    crop = table.getDoubleArrayTopic("crop").publish();
    cameraPoseRobotSpacePub = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();

    priorityId = table.getIntegerTopic("priorityid").publish();
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

  private LimelightVisionFrame getLimelightVisionFrame(double[] arr) {
    return LimelightVisionFrame.builder()
        .translationX(arr[0])
        .translationY(arr[1])
        .translationZ(arr[2])
        .rotationRoll(arr[3])
        .rotationPitch(arr[4])
        .rotationYaw(arr[5])
        .totalLatency(arr[6])
        .tagCount(arr[7])
        .tagSpan(arr[8])
        .averageTagDistanceFromCamera(arr[9])
        .averageTagArea(arr[10])
        .build();
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    var lvf = getLimelightVisionFrame(botPoseWpiBlue.get());

    inputs.botPoseBlue =
        new Pose3d(
            new Translation3d(lvf.translationX, lvf.translationY, lvf.translationZ),
            new Rotation3d(lvf.rotationRoll, lvf.rotationPitch, lvf.rotationYaw));
    inputs.botPoseBlueTimestamp = Timer.getFPGATimestamp() - lvf.totalLatency;

    inputs.hasTarget = tv.get() == 1;
    inputs.horizontalOffsetFromTarget = tx.get();
    inputs.verticalOffsetFromTarget = ty.get();
    inputs.targetArea = ta.get();
    inputs.pipelineLatencyMs = tl.get();
    inputs.captureLatencyMs = cl.get();
    inputs.activePipeline = getpipe.get();
    inputs.tagCount = lvf.tagCount;
    inputs.tagSpan = lvf.tagSpan;
    inputs.averageTagDistanceFromCamera = lvf.averageTagDistanceFromCamera;
    inputs.averageTagArea = lvf.averageTagArea;
    inputs.tagId = aprilTagId.get();
  }

  @Override
  public VisionInfo getInfo() {
    return info;
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    ledMode.set(
        switch (mode) {
          case PIPELINE -> 0;
          case FORCE_OFF -> 1;
          case FORCE_BLINK -> 2;
          case FORCE_ON -> 3;
        });
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

  public void setPriorityId(int tagId) {
    priorityId.set(tagId);
  }
}
