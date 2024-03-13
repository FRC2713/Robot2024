package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class BottomTwoBlue extends SequentialCommandGroup {
  private ChoreoTrajectory traj1, traj2, traj3;

  public BottomTwoBlue() {
    traj1 = Choreo.getTrajectory("Bottom Two.1");
    traj2 = Choreo.getTrajectory("Bottom Two.2");
    traj3 = Choreo.getTrajectory("Bottom Two.3");

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        new ParallelCommandGroup(
            ShootingCommands.runPathAndIntake(traj1),
            Commands.sequence(
                new WaitCommand(2),
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.DYNAMIC_AIM))),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Second Piece
        new ParallelCommandGroup(
            ShootingCommands.runPathAndIntake(traj2),
            Commands.sequence(
                new WaitCommand(2),
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.DYNAMIC_AIM))),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Shooter.Commands.setState(Shooter.State.OFF),
        Intake.Commands.setMotionMode(Intake.State.OFF),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Clear out
        ShootingCommands.runPath(traj3));
  }
}
