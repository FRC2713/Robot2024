package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;

public class Simple extends SequentialCommandGroup {
  public Simple() {
    // addCommands(
    // new SequentialCommandGroup(
    // new InstantCommand(
    // () -> {
    // Robot.swerveDrive.resetOdometry(
    // Autos.FIVE_TO_B.getTrajectory().getInitialHolonomicPose());
    // }),
    //
    // SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.FIVE_TO_B.getTrajectory())));

    PathPlannerPath path = PathPlannerPath.fromPathFile("Simple");

    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(path.getPreviewStartingHolonomicPose());
              Robot.swerveDrive.setMotionMode(MotionMode.TRAJECTORY);
              // Robot.swerveDrive.resetOdometry(path.get);
            }),
        AutoBuilder.followPath(path));
  }
}
