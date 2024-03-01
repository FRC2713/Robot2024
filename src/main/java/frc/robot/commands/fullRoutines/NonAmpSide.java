package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class NonAmpSide extends SequentialCommandGroup {

  public NonAmpSide() {
    addCommands(
        SwerveSubsystem.Commands.resetOdometry("Non Amp Side.1"),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        new WaitUntilCommand(Robot.shooterPivot::isAtTargetAngle),
        ShootingCommands.runShooter(Shooter.State.FENDER_SHOT),

        // First Piece
        ShootingCommands.runPathAndIntake("Non Amp Side.1"),
        ShootingCommands.runPathAndShoot(
            "Non Amp Side.2",
            Shooter.State.AUTO_SHOT_NonAmpSide_1,
            ShooterPivot.State.AUTO_SHOT_NonAmpSide_1),

        // Second Piece
        ShootingCommands.runPathAndIntake("Non Amp Side.3"),
        ShootingCommands.runPathAndShoot(
            "Non Amp Side.4",
            Shooter.State.AUTO_SHOT_NonAmpSide_2,
            ShooterPivot.State.AUTO_SHOT_NonAmpSide_2),
        new InstantCommand(
            () -> {
              SwerveSubsystem.Commands.errorTracker.printSummary("NonAmpSide");
            }));
  }
}
