package frc.robot.rhr.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ErrorTracker;
import org.littletonrobotics.junction.Logger;

public class RHRPathPlannerAuto extends PathPlannerAuto {

  ErrorTracker errorTracker;

  public RHRPathPlannerAuto(String autoName) {
    this(autoName, 10);
  }

  public RHRPathPlannerAuto(String autoName, int errorTrackerSamples) {
    super(autoName);
    this.errorTracker =
        new ErrorTracker(
            errorTrackerSamples,
            Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_X,
            Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    errorTracker.printSummary(this.getName());
  }

  @Override
  public void initialize() {
    super.initialize();
    errorTracker.reset();

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Pose2d currentPose = Robot.swerveDrive.getRegularPose();
          var error =
              new Pose2d(
                  new Translation2d(
                      targetPose.getX() - currentPose.getX(),
                      targetPose.getY() - currentPose.getY()),
                  Rotation2d.fromRadians(
                      targetPose.getRotation().getRadians()
                          - currentPose.getRotation().getRadians()));

          errorTracker.addObservation(error);
          Logger.recordOutput("PathPlanner/Target Pose", targetPose);
          Logger.recordOutput("PathPlanner/Pose Error", error);
        });
  }
}
