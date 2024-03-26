package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class AmpSideLong extends RHRFullRoutine {

  private ChoreoTrajectory traj2, traj3;

  public AmpSideLong() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.3"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj1, Shooter.State.PODIUM_SHOT_NO_FEEDER, ShooterPivot.State.CLUTCH_AUTO_1),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.PODIUM_SHOT, ShooterPivot.State.CLUTCH_AUTO_1),
        RedHawkUtil.logShot(),
        new WaitCommand(0.1),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj2, Shooter.State.PODIUM_SHOT_NO_FEEDER, ShooterPivot.State.CLUTCH_AUTO_2),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.PODIUM_SHOT, ShooterPivot.State.CLUTCH_AUTO_2),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Third Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj3, Shooter.State.PODIUM_SHOT_NO_FEEDER, ShooterPivot.State.CLUTCH_AUTO_3),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.PODIUM_SHOT, ShooterPivot.State.CLUTCH_AUTO_3),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Cmds.setState(Shooter.State.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),
        Cmds.setState(Intake.State.OFF));
  }
}
