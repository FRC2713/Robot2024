package frc.robot.commands.fullRoutines;

import com.choreo.lib.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class ThreePieceChoreo extends SequentialCommandGroup {

  public ThreePieceChoreo() {
    ChoreoTrajectory firstTraj = Choreo.getTrajectory("3 Piece Choreo.1"); //
    ChoreoTrajectory secondTraj = Choreo.getTrajectory("3 Piece Choreo.2");
    ChoreoTrajectory thirdTraj = Choreo.getTrajectory("3 Piece Choreo.3");
    //
    addCommands(
        new InstantCommand(
            () -> {
              SwerveSubsystem.Commands.errorTracker.reset();
              Robot.swerveDrive.resetOdometry(firstTraj.getInitialPose());
            }),
        // ShootingCommands.FeederShotCommands(),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.choreoCommandBuilder(firstTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        new WaitCommand(1),
        // ShootingCommands.FullShotCommands(),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.choreoCommandBuilder(secondTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        new WaitCommand(1),
        // ShootingCommands.FullShotCommands(),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.choreoCommandBuilder(thirdTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        new WaitCommand(1),
        // ShootingCommands.FullShotCommands(),
        new InstantCommand(
            () -> {
              SwerveSubsystem.Commands.errorTracker.printSummary("ThreePieceChoreo");
            }));
  }
}
