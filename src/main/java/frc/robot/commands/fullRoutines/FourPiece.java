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

public class FourPiece extends SequentialCommandGroup {

  private ChoreoTrajectory traj1, traj2, traj3, traj4;

  public FourPiece() {
    traj1 = Choreo.getTrajectory("Four Piece.1").flipped();
    traj2 = Choreo.getTrajectory("Four Piece.2").flipped();
    traj3 = Choreo.getTrajectory("Four Piece.3").flipped();
    traj4 = Choreo.getTrajectory("Four Piece.4").flipped();

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),
        RedHawkUtil.logShot(),

        // First Piece
        ShootingCommands.runPathAndIntake(traj1),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.FENDER_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPathAndIntake(traj2),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.FENDER_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Third Piece
        ShootingCommands.runPathAndIntake(traj3),
        new WaitCommand(0.7),
        ShootingCommands.runShooterAndPivot(
            Shooter.State.FENDER_SHOT, ShooterPivot.State.DYNAMIC_AIM),
        RedHawkUtil.logShot(),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Shooter.Commands.setState(Shooter.State.OFF),
        Intake.Commands.setMotionMode(Intake.State.OFF),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING),

        // Go for 4
        ShootingCommands.runPathAndIntake(traj4));
  }
}