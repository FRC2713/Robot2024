package frc.robot.subsystems.visionIO;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionLimelight implements VisionIO {

  String name;
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

  public VisionLimelight(String name) {
    this.name = name;
    table = NetworkTableInstance.getDefault().getTable(name);

    botPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    botPoseWpiBlue = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    botPoseWpiRed = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[] {});
    cameraPoseTargetSpace =
        table.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});
    targetPoseCameraSpace =
        table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    targetPoseRobotSpace =
        table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[] {});
    botPoseTargetSpace =
        table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[] {});
    cameraPoseRobotSpace =
        table.getDoubleArrayTopic("camerapose_robotspace").subscribe(new double[] {});
    tc = table.getDoubleArrayTopic("tc").subscribe(new double[] {});

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
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.botPose = botPose.get();
    inputs.botPoseBlue = botPoseWpiBlue.get();
    inputs.botPoseRed = botPoseWpiRed.get();
    inputs.cameraPoseInTargetSpace = cameraPoseTargetSpace.get();
    inputs.targetPoseInCameraSpace = targetPoseCameraSpace.get();
    inputs.targetPoseInRobotSpace = targetPoseRobotSpace.get();
    inputs.botPoseInTargetSpace = botPoseTargetSpace.get();
    inputs.cameraPoseInRobotSpace = cameraPoseRobotSpace.get();

    inputs.averageHsvColor = tc.get();
    inputs.aprilTagId = aprilTagId.get();
    inputs.hasValidTarget = tv.get() == 1;
    inputs.horizontalOffsetFromTarget = tx.get();
    inputs.verticalOffsetFromTarget = ty.get();
    inputs.targetArea = ta.get();
    inputs.pipelineLatency = tl.get();
    inputs.captureLatency = cl.get();
    inputs.totalLatency = inputs.pipelineLatency + inputs.captureLatency;
    inputs.shortestBoundingBoxSidelength = tshort.get();
    inputs.longestBoundingBoxSideLength = tlong.get();
    inputs.horizontalBoundingBoxSideLength = thor.get();
    inputs.verticalBoundingBoxSideLength = tvert.get();
    inputs.activePipeline = getpipe.get();
    inputs.neuralNetClassId = tclass.get();
  }

  @Override
  public String getName() {
    return name;
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
}
