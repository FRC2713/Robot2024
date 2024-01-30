package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.rhr.auto.RHRPathPlannerAuto;

public class SimpleChoreo {

  public static Command getAutonomousCommand() {
    PathPlannerPath p = PathPlannerPath.fromChoreoTrajectory("Simple Choreo");
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(p.getPreviewStartingHolonomicPose());
            }),
        new RHRPathPlannerAuto("Simple Choreo Auto"));
  }
}
