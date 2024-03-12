package frc.robot.subsystems.visionIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.visionIO.VisionIO.LEDMode;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionInputsAutoLogged inputs;
  private final String key;

  public Vision(VisionIO io) {
    this.io = io;
    this.key = String.format("Vision/%s", io.getInfo().getNtTableName());
    inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
  }

  private String outputKey(String name) {
    return String.format("%s/%s", this.key, name);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    Logger.recordOutput(outputKey("Bot Pose (2d)"), inputs.botPoseBlue.toPose2d());
  }

  @AutoLogOutput(key = "Vision/estimate")
  public double estimateDistanceToTag() {
    double angleToTagDegrees =
        Constants.DynamicShooterConstants.limelightMouningAngleDegrees
            + this.getInputs().verticalOffsetFromTarget;
    double angleToTagRadians = angleToTagDegrees * (Math.PI / 180.0);
    return (Constants.DynamicShooterConstants.tagMountingHeight
            - Constants.DynamicShooterConstants.limelightMountingHeightMeters)
        / Math.tan(angleToTagRadians);
  }

  public VisionInfo getInfo() {
    return io.getInfo();
  }

  public VisionInputs getInputs() {
    return inputs;
  }

  public void setLEDMode(LEDMode leds) {
    io.setLEDMode(leds);
  }

  public void setPriorityId(int tagId) {
    io.setPriorityId(tagId);
  }
}
