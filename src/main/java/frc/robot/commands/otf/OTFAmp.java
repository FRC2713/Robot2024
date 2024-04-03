package frc.robot.commands.otf;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.util.ErrorTracker;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class OTFAmp {
  public static OTFAmp INSTANCE;

  private OTFAmp() {}

  public static OTFAmp getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new OTFAmp();
    }
    return INSTANCE;
  }

  public Timer timer = new Timer();
  public double ttl = 3;
  private Pose2d ampPose = new Pose2d(1.8, 7.67, Rotation2d.fromDegrees(90));
  private Pose2d preAmpPose =
      new Pose2d(1.8, 7.67 - Units.inchesToMeters(2), Rotation2d.fromDegrees(90));

  @Getter
  public ErrorTracker tracker =
      new ErrorTracker(
          10,
          Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_X,
          Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION);

  private Command runningCommand;

  public Command run() {
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Pose2d currentPose = Robot.swerveDrive.getEstimatedPose();
          var error =
              new Pose2d(
                  new Translation2d(
                      targetPose.getX() - currentPose.getX(),
                      targetPose.getY() - currentPose.getY()),
                  Rotation2d.fromRadians(
                      targetPose.getRotation().getRadians()
                          - currentPose.getRotation().getRadians()));

          getTracker().addObservation(error);
          Logger.recordOutput("PathPlanner/Target Pose", targetPose);
          Logger.recordOutput("PathPlanner/Pose Error", error);
        });

    PathConstraints constraints =
        new PathConstraints(
            OTF.OTFOptions.AMP_STATIC.maxVelocityMps,
            OTF.OTFOptions.AMP_STATIC.maxAngularAccelerationRpsSq,
            OTF.OTFOptions.AMP_STATIC.maxAngularVelocityRps,
            OTF.OTFOptions.AMP_STATIC.maxAngularAccelerationRpsSq);

    List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(Robot.swerveDrive.getEstimatedPose(), ampPose);

    // Create the path using the bezier points created above
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            constraints, // The constraints for this path. If using a
            // differential drivetrain, the angular constraints
            // have no effect.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    -90)) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    runningCommand =
        Commands.sequence(
            SwerveSubsystem.Commands.setHeading(Rotation2d.fromDegrees(-90)),
            AutoBuilder.followPath(path));
    return runningCommand;
  }

  public Command runAndRegenerate() {
    return Commands.sequence(
        new InstantCommand(() -> Robot.swerveDrive.setMotionMode(MotionMode.TRAJECTORY)),
        new InstantCommand(
            () -> {
              timer.reset();
              timer.start();
            }),
        Commands.parallel(run(), new RepeatCommand(maybeRegenerateTraj())));
  }

  public Command maybeRegenerateTraj() {
    Logger.recordOutput("OTF/TimerSeconds", timer.get());
    Logger.recordOutput("OTF/TimeToRegeneration", ttl - timer.get());
    if (timer.hasElapsed(ttl)) {
      return run();
    }
    return new InstantCommand();
  }

  public void cancelCommand() {
    timer.reset();
    if (runningCommand != null) {
      runningCommand.cancel();
    }
  }

  public void printErrorSummary() {
    getTracker().printSummary("OTF/AMP_STATIC");
  }
}
