package frc.robot.subsystems.visionIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionInputsAutoLogged inputs;
  private final String key;

  public Vision(String key, VisionIO io) {
    this.io = io;
    this.key = String.format("Vision/%s", key);
    inputs = new VisionInputsAutoLogged();
    io.updateInputs(inputs);
  }

  private String outputKey(String name) {
    return String.format("%s/%s", this.key, name);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    Logger.recordOutput(outputKey("Bot Pose (2d)"), inputs.botPose.toPose2d());
  }
}
