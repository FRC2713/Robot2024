package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class BottomTwo extends RHRFullRoutine {
  private ChoreoTrajectory traj2, traj3;

  public BottomTwo() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.3"));

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        ShootingCommands.runPathAndIntake(traj1),
        ShootingCommands.runShooterAndPivot(
            ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPathAndIntake(traj2),
        ShootingCommands.runShooterAndPivot(
            ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Cmds.setState(Shooter.ShooterState.OFF),
        Cmds.setState(Intake.State.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Clear out
        ShootingCommands.runPath(traj3));
  }
}
