package frc.robot.commands.otf;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ErrorTracker;
import org.littletonrobotics.junction.Logger;

public class OTF {
  ErrorTracker tracker =
      new ErrorTracker(
          10,
          Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_X,
          Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION);

  Command runningCommand;

  public ErrorTracker getTracker() {
    return tracker;
  }

  public Command followPath() {
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Pose2d currentPose = Robot.swerveDrive.getUsablePose();
          var error =
              new Pose2d(
                  new Translation2d(
                      targetPose.getX() - currentPose.getX(),
                      targetPose.getY() - currentPose.getY()),
                  Rotation2d.fromRadians(
                      targetPose.getRotation().getRadians()
                          - currentPose.getRotation().getRadians()));

          tracker.addObservation(error);
          Logger.recordOutput("PathPlanner/Target Pose", targetPose);
          Logger.recordOutput("PathPlanner/Pose Error", error);
        });

    String autoName = "First Piece";
    PathPlannerPath path = PathPlannerPath.fromPathFile(autoName);
    NamedCommands.registerCommand(
        "FireOuttake", new InstantCommand(() -> Logger.recordOutput("Firing", true)));

    var x = Robot.swerveDrive.getUsablePose().getX();
    var alliance =
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Blue;
    // if ((alliance == DriverStation.Alliance.Blue && x > 5.9)
    //     || (alliance != DriverStation.Alliance.Blue && x < 10.4)) {
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand =
        AutoBuilder.pathfindThenFollowPath(
            path,
            constraints,
            0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            );

    runningCommand = pathfindingCommand;
    return runningCommand;
    // } else {
    // runningCommand = AutoBuilder.followPath(path);
    // return runningCommand;
    // }
  }

  public void cancelCommand() {
    runningCommand.cancel();
  }

  public OTF() {}
}
