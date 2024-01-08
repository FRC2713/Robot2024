package frc.robot.subsystems.visionIO;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.visionIO.Vision.SnapshotMode;
import frc.robot.util.LimelightHelpers;

public class VisionLimelight implements VisionIO {
  String name;

  public VisionLimelight(String name) {
    this.name = name;
  }

  private NetworkTable getTable() {
    return NetworkTableInstance.getDefault().getTable(name);
  }

  private NetworkTableEntry getEntry(String entryName) {
    return getTable().getEntry(entryName);
  }

  private double getValueDouble(String entryName) {
    return getEntry(entryName).getDouble(0);
  }

  private boolean getValueBoolean(String entryName) {
    return getEntry(entryName).getBoolean(false);
  }

  private long getValueLong(String entryName) {
    return getEntry(entryName).getInteger(0);
  }

  private String getValueString(String entryName) {
    return getEntry(entryName).getString("");
  }

  private double[] getValueDoubleArray(String entryName) {
    return getEntry(entryName).getDoubleArray(new double[] {});
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.validTarget = getValueBoolean("tv");
    inputs.horizontalCrosshairOffset = getValueDouble("tx");
    inputs.verticalCrosshairOffset = getValueDouble("ty");
    inputs.targetArea = getValueDouble("ta");
    inputs.skew = getValueDouble("ts");
    inputs.pipelineLatency = getValueDouble("tl");
    inputs.shortSidelength = getValueDouble("tshort");
    inputs.longSideLength = getValueDouble("tlong");
    inputs.horizontalSideLength = getValueDouble("thor");
    inputs.verticalSideLength = getValueDouble("tvert");
    inputs.pipelineIndex = getValueLong("getpipe");
    inputs.tid = getValueLong("tid");
    inputs.jsonDump = getValueString("json");
    inputs.botpose = getValueDoubleArray("botpose");
    inputs.botpose_wpiblue = getValueDoubleArray("botpose_wpiblue");
    inputs.botpose_wpired = getValueDoubleArray("botpose_wpired");
    inputs.camerapose_targetspace = getValueDoubleArray("camerapose_targetspace");
    inputs.targetpose_cameraspace = getValueDoubleArray("targetpose_cameraspace");
    inputs.targetpose_robotspace = getValueDoubleArray("targetpose_robotspace");
    inputs.botpose_targetspace = getValueDoubleArray("botpose_targetspace");
    inputs.camerapose_robotspace = getValueDoubleArray("camerapose_robotspace");
    inputs.neuralDetectorID = getValueLong("tclass");
    inputs.ledMode = getValueLong("ledMode");
    inputs.camMode = getValueLong("camMode");
    inputs.pipeline = getValueLong("pipeline");
    inputs.stream = getValueLong("stream");
    inputs.crop = getValueDoubleArray("crop");
    var results = LimelightHelpers.getLatestResults(this.name);
    if (results != null && results.targetingResults != null && results.targetingResults.valid) {
      inputs.numTargets = results.targetingResults.targets_Fiducials.length;
    }
  }

  /**
   * @param mode The LED Mode to set on the Limelight
   */
  @Override
  public void setSnapshotMode(SnapshotMode mode) {
    if (mode != SnapshotMode.UNKNOWN) {
      NetworkTableInstance.getDefault()
          .getTable("limelight")
          .getEntry("snapshot")
          .setNumber(mode.value);
    }
  }

  @Override
  public String getName() {
    // TODO Auto-generated method stub
    return name;
  }
}
