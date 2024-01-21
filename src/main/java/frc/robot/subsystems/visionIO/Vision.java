package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
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

  private Optional<Pair<Pose3d, Double>> extractPoseAndTime(double[] limelightPoseArray) {
    if (limelightPoseArray.length == 0) {
      return Optional.empty();
    }

    return Optional.of(
        new Pair<Pose3d, Double>(
            new Pose3d(
                new Translation3d(
                    limelightPoseArray[0], limelightPoseArray[1], limelightPoseArray[2]),
                new Rotation3d(
                    Units.degreesToRadians(limelightPoseArray[3]),
                    Units.degreesToRadians(limelightPoseArray[4]),
                    Units.degreesToRadians(limelightPoseArray[5]))),
            limelightPoseArray[6]));
  }

  private String outputKey(String name) {
    return String.format("%s/%s", this.key, name);
  }

  private void logOptionalPose(String name, double[] poseArray) {
    var timedPose = extractPoseAndTime(poseArray);
    Logger.recordOutput(outputKey(name + "/Has Pose"), timedPose.isPresent());

    if (timedPose.isPresent()) {
      Logger.recordOutput(outputKey(name + "/Pose"), timedPose.get().getFirst());
      Logger.recordOutput(outputKey(name + "/Time"), timedPose.get().getSecond());
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    logOptionalPose("Bot Pose (blue)", inputs.botPoseBlue);
  }
}
