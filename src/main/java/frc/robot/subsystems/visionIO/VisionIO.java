package frc.robot.subsystems.visionIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionInputs implements LoggableInputs {
    public Pose2d botPoseBlue = new Pose2d();
    public double botPoseBlueTimestamp = 0;
    public boolean hasTarget = false;
    public double tagCount = 0;
    public double totalLatencyMs = 0;
    public double averageTagDistanceFromCamera = 0;
    public double averageTagArea = 0;

    public LimelightResults results;

    public static String[][] stringTitles;

    static {
      int size = 1000;
      stringTitles = new String[4][size];
      for (int i = 0; i < size; i++) {
        stringTitles[0][i] = "Targets/" + i + "/tx";
        stringTitles[1][i] = "Targets/" + i + "/ty";
        stringTitles[2][i] = "Targets/" + i + "/ta";
        stringTitles[3][i] = "Targets/" + i + "/ts";
      }
    }

    @Override
    public void toLog(LogTable table) {

      table.put("BotPoseBlue", botPoseBlue);
      table.put("BotPoseBlueTimestamp", botPoseBlueTimestamp);
      table.put("HasTarget", hasTarget);
      table.put("TagCount", tagCount);
      table.put("TotalLatencyMs", totalLatencyMs);
      table.put("AverageTagDistanceFromCamera", averageTagDistanceFromCamera);
      table.put("AverageTagArea", averageTagArea);

      var targets = results.targetingResults.targets_Fiducials;
      for (int i = 0; i < targets.length; i++) {
        int fiducialID = (int) targets[i].fiducialID;
        table.put(stringTitles[0][fiducialID], targets[i].tx);
        table.put(stringTitles[1][fiducialID], targets[i].ty);
        table.put(stringTitles[2][fiducialID], targets[i].ta);
        table.put(stringTitles[3][fiducialID], targets[i].ts);
      }
    }

    @Override
    public void fromLog(LogTable table) {}

    public Optional<LimelightTarget_Fiducial> getByTagId(double tagId) {
      for (var x : this.results.targetingResults.targets_Fiducials) {
        if (x.fiducialID == tagId) {
          return Optional.of(x);
        }
      }

      return Optional.empty();
    }
  }

  public void updateInputs(VisionInputs inputs);

  public VisionInfo getInfo();

  public enum LEDMode {
    PIPELINE,
    FORCE_OFF,
    FORCE_BLINK,
    FORCE_ON
  };

  public enum CameraMode {
    VISION,
    DRIVER_CAM
  };

  public enum StreamMode {
    STANDARD,
    PIP_MAIN,
    PIP_SECONDARY
  };

  public enum SnapshotMode {
    NONE,
    TAKE_ONE
  };

  public void setLEDMode(LEDMode mode);

  public void setCameraMode(CameraMode mode);

  public void setPipeline(int pipeline);

  public void setStreamMode(StreamMode streamMode);

  public void setSnapshotMode(SnapshotMode mode);

  public void setCrop(double x0, double x1, double y0, double y1);

  public void setCameraPoseInRobotSpaceInternal(
      double forward, double side, double up, double roll, double pitch, double yaw);

  public default void setCameraPoseInRobotSpace(Pose3d pose) {
    setCameraPoseInRobotSpaceInternal(
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  public void setPriorityId(int tagId);
}
