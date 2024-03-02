package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class NonAmpSide extends SequentialCommandGroup {

  private ChoreoTrajectory traj1, traj2;

  public NonAmpSide() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.2"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        new WaitUntilCommand(Robot.shooterPivot::isAtTargetAngle),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),

        // First Piece
        ShootingCommands.runPathAndIntake(traj1),
        ShootingCommands.runPathAndShoot(
            traj2,
            Shooter.State.AUTO_SHOT_NonAmpSide_1,
            ShooterPivot.State.AUTO_SHOT_NonAmpSide_1));

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
