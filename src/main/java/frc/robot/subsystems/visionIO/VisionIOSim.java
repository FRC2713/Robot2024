package frc.robot.subsystems.visionIO;

public class VisionIOSim implements VisionIO {

  public String getName() {
    return "";
  }

  @Override
  public void updateInputs(VisionInputs inputs) {}

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
