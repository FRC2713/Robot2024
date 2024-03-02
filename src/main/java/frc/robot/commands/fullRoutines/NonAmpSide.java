package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class NonAmpSide extends SequentialCommandGroup {

  private ChoreoTrajectory traj1;

  public NonAmpSide() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.1"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),
        SwerveSubsystem.Commands.choreoCommandBuilder(traj1));

    // Preload
    // ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
    // new WaitUntilCommand(Robot.shooterPivot::isAtTargetAngle),
    // ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),

    // First Piece
    // ShootingCommands.runPathAndIntake(traj1));
    // ShootingCommands.runPathAndShoot(
    //     "Non Amp Side.2",
    //     Shooter.State.AUTO_SHOT_NonAmpSide_1,
    //     ShooterPivot.State.AUTO_SHOT_NonAmpSide_1),

    // // Second Piece
    // ShootingCommands.runPathAndIntake("Non Amp Side.3"),
    // ShootingCommands.runPathAndShoot(
    //     "Non Amp Side.4",
    //     Shooter.State.AUTO_SHOT_NonAmpSide_2,
    //     ShooterPivot.State.AUTO_SHOT_NonAmpSide_2),
    // new InstantCommand(
    //     () -> {
    //       SwerveSubsystem.Commands.errorTracker.printSummary("NonAmpSide");
    //     }));
  }
}
