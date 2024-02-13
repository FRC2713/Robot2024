package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class Week0MobilityChoreo extends SequentialCommandGroup {
  public Week0MobilityChoreo() {

    ChoreoTrajectory Traj = Choreo.getTrajectory("Week 0 Mobility.1");

    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(Traj.getInitialPose());
            }),
        ShootingCommands.FeederShotCommands(),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.choreoCommandBuilder(Traj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        ShootingCommands.FullShotCommands(),
        new InstantCommand(
            () -> {
              SwerveSubsystem.Commands.errorTracker.printSummary("Week0MobilityChoreo");
            }));
  }
}
