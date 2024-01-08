package frc.robot.subsystems.visionIO;

import frc.robot.subsystems.visionIO.Vision.SnapshotMode;

public class VisionIOSim implements VisionIO {

  public String getName() {
    return "";
  }

  @Override
  public void updateInputs(VisionInputs inputs) {}

  @Override
  public void setSnapshotMode(SnapshotMode mode) {}
}
