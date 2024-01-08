package frc.robot.rhr.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.RedHawkUtil;
import java.util.List;

public class RHRTrajectory {
  private final List<RHRTrajectoryState> states;

  public RHRTrajectory(List<RHRTrajectoryState> states) {
    this.states = states;
  }

  public RHRTrajectoryState sample(double timestamp) {
    RHRTrajectoryState before = null;
    RHRTrajectoryState after = null;

    for (RHRTrajectoryState state : this.states) {
      // If requested time is equal to one of the state times, we have found the right state
      if (timestamp == state.timestamp()) {
        return state;
      }

      if (state.timestamp() < timestamp) {
        before = state;
      } else {
        after = state;
        break;
      }
    }

    if (before == null) {
      return states.get(0);
    }

    if (after == null) {
      return states.get(states.size() - 1);
    }

    double s = 1 - ((after.timestamp() - timestamp) / (after.timestamp() - before.timestamp()));

    return new RHRTrajectoryState(
        timestamp,
        RedHawkUtil.lerp(before.pose(), after.pose(), s),
        RedHawkUtil.lerp(before.chassisSpeeds(), after.chassisSpeeds(), s));
  }

  public double getDuration() {
    return states.get(states.size() - 1).timestamp();
  }

  public static RHRTrajectory fromPathPlanner(PathPlannerTrajectory ppTrajectory) {
    return new RHRTrajectory(
        ppTrajectory.getStates().stream()
            .map(state -> (PathPlannerState) state)
            .map(
                state ->
                    new RHRTrajectoryState(
                        state.timeSeconds,
                        new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation),
                        new ChassisSpeeds(
                            state.velocityMetersPerSecond * state.poseMeters.getRotation().getCos(),
                            state.velocityMetersPerSecond * state.poseMeters.getRotation().getSin(),
                            state.holonomicAngularVelocityRadPerSec)))
            .toList());
  }
}
