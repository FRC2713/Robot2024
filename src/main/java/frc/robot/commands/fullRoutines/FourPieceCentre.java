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

public class FourPieceCentre extends RHRFullRoutine {

  private ChoreoTrajectory traj2, traj3, traj4;

  public FourPieceCentre() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Four Piece Centre.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Four Piece Centre.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Four Piece Centre.3"));
    traj4 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Four Piece Centre.4"));

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        ShootingCommands.runPathAndIntake(traj1),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.NO_DIFFERENTIAL_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPathAndIntake(traj2),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.NO_DIFFERENTIAL_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Third Piece
        ShootingCommands.runPathAndIntake(traj3),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.ShooterState.NO_DIFFERENTIAL_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Cmds.setState(Shooter.ShooterState.OFF),
        Cmds.setState(Intake.State.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Go for 4
        ShootingCommands.runPathAndIntake(traj4));
  }
}
