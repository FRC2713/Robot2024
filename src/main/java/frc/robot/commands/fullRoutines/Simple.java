package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

public class Simple extends SequentialCommandGroup {
  public static Command getAutonomousCommand() {
    NamedCommands.registerCommand(
        "Simple NamedCommand",
        new InstantCommand(
            () -> {
              Logger.recordOutput("DidSOMETHING", true);
            }));
    return new SequentialCommandGroup(new PathPlannerAuto("Simple Auto"));
  }
  // public Simple() {
  // addCommands(
  // new SequentialCommandGroup(
  // new InstantCommand(
  // () -> {
  // Robot.swerveDrive.resetOdometry(
  // Autos.FIVE_TO_B.getTrajectory().getInitialHolonomicPose());
  // }),
  //
  // SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.FIVE_TO_B.getTrajectory())));

  // PathPlannerAuto auto = new PathPlannerAuto("Simple Auto");
  // PathPlannerPath path = PathPlannerPath.fromPathFile("Simple Path");

  // addCommands(
  // new InstantCommand(
  // () -> {
  // // Robot.swerveDrive.resetOdometry(
  // // PathPlannerAuto.getPathGroupFromAutoFile("Simple")
  // // .get(1)
  // // .getPreviewStartingHolonomicPose());
  // Robot.swerveDrive.resetOdometry(path.getPreviewStartingHolonomicPose());
  // Robot.swerveDrive.setMotionMode(MotionMode.TRAJECTORY);
  // // Robot.swerveDrive.resetOdometry(path.get);
  // }),
  // AutoBuilder.followPath(path));
  // AutoBuilder.followPath(PathPlannerAuto.getPathGroupFromAutoFile("Simple").get(1)));
  // }
}
