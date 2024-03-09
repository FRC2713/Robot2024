package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class NonAmpSide extends SequentialCommandGroup {

  private ChoreoTrajectory traj1, traj2, traj3;

  public NonAmpSide() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.3"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        ShootingCommands.runPath(traj1),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.FENDER_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPath(traj2),
        new WaitCommand(0.3),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.FENDER_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Shooter.Commands.setState(Shooter.State.OFF),
        Intake.Commands.setMotionMode(Intake.State.OFF),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Go for 3
        ShootingCommands.runPath(traj3));
  }
}
