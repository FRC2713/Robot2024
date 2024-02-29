package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class NonAmpSide extends SequentialCommandGroup {

  public NonAmpSide() {
    addCommands(
        SwerveSubsystem.Commands.resetOdometry("Non Amp Side.1"),
        ShootingCommands.runPathAndIntake("Non Amp Side.1"),
        ShootingCommands.runPathAndShoot(
            "Non Amp Side.2",
            Shooter.State.AUTO_SHOT_NonAmpSide_1,
            ShooterPivot.State.AUTO_SHOT_NonAmpSide_1),
        ShootingCommands.runPathAndIntake("Non Amp Side.3"),
        ShootingCommands.runPathAndShoot(
            "Non Amp Side.4",
            Shooter.State.AUTO_SHOT_NonAmpSide_2,
            ShooterPivot.State.AUTO_SHOT_NonAmpSide_2),

        // new ParallelCommandGroup(
        //     SwerveSubsystem.Commands.choreoCommandBuilder(firstTraj),
        //     Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)),
        // ShootingCommands.FullShotCommands(),
        // new ParallelCommandGroup(
        //     SwerveSubsystem.Commands.choreoCommandBuilder(secondTraj),
        //     Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)),
        // ShootingCommands.FullShotCommands(),
        // new ParallelCommandGroup(
        //     SwerveSubsystem.Commands.choreoCommandBuilder(thirdTraj),
        //     Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)),
        // ShootingCommands.FullShotCommands(),
        new InstantCommand(
            () -> {
              SwerveSubsystem.Commands.errorTracker.printSummary("NonAmpSide");
            }));
  }
}
