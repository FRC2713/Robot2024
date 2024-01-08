package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Simple extends SequentialCommandGroup {
  public Simple() {
    // addCommands(
    //     new SequentialCommandGroup(
    //         new InstantCommand(
    //             () -> {
    //               Robot.swerveDrive.resetOdometry(
    //                   Autos.FIVE_TO_B.getTrajectory().getInitialHolonomicPose());
    //             }),
    //
    // SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.FIVE_TO_B.getTrajectory())));
  }
}
