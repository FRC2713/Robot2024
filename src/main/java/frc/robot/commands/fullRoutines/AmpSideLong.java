package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class AmpSideLong extends RHRFullRoutine {

  private ChoreoTrajectory traj2, traj3, traj4;

  public AmpSideLong() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.3"));
    traj4 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Amp Side Long.4"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        // Set shooter on always
        Cmds.setState(ShooterState.DIFFERENTIAL_SHOT),

        // First Piece
        ShootingCommands.runPathAndIntake(traj1),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),

        // Second Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj2, ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.CLUTCH_AUTO_1),
        ShootingCommands.runShooterAndPivot(
            ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.CLUTCH_AUTO_1),
        RedHawkUtil.logShot(),
        new WaitCommand(0.1),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Third Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj3, Shooter.ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.CLUTCH_AUTO_2),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.CLUTCH_AUTO_2),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Fourth Piece
        ShootingCommands.runPathAndIntake(traj4),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.CLUTCH_AUTO_3),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Cmds.setState(Shooter.ShooterState.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),
        Cmds.setState(Intake.State.OFF));
  }
}
