package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.FeederState;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class ShootingCommands {
  public static Command runPath(ChoreoTrajectory choreoPath) {
    return SwerveSubsystem.Commands.choreoCommandBuilder(choreoPath);
  }

  public static Command runPathAndIntake(ChoreoTrajectory choreoPath) {
    return new SequentialCommandGroup(
        ShootingCommands.runIntake(), ShootingCommands.runPath(choreoPath));
  }

  public static Command runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
      ChoreoTrajectory choreoPath,
      Shooter.ShooterState shooterState,
      ShooterPivot.State shooterPivotState) {
    return new ParallelDeadlineGroup(
        ShootingCommands.runPathAndIntake(choreoPath),
        Commands.either(
            new PrintCommand("Already have GP"),
            Commands.sequence(
                new WaitUntilCommand(() -> Robot.shooter.hasGamePiece()),
                Cmds.setState(Intake.State.OFF),
                Cmds.setState(shooterState),
                ShootingCommands.runShooterPivot(shooterPivotState)),
            () -> Robot.shooter.hasGamePiece()));
  }

  public static Command runPathAndShoot(
      ChoreoTrajectory choreoPath,
      ShooterState shooterState,
      ShooterPivot.State shooterPivotState) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            ShootingCommands.runShooterPivot(shooterPivotState),
            ShootingCommands.runPath(choreoPath)),
        ShootingCommands.runShooter(shooterState));
  }

  public static Command runShooterAndPivot(
      ShooterState shooterState, ShooterPivot.State shooterPivotState) {
    return new SequentialCommandGroup(
        ShootingCommands.runShooterPivot(shooterPivotState),
        new WaitUntilCommand(() -> Robot.shooterPivot.isAtTargetAngle()),
        ShootingCommands.runShooter(shooterState));
  }

  public static Command runIntake() {
    return Commands.sequence(
        Cmds.setState(Intake.State.INTAKE_GP),
        Cmds.setState(FeederState.INTAKE),
        Cmds.setState(ShooterPivot.State.INTAKING));
  }

  public static Command runShooter(ShooterState shooterState) {
    return new SequentialCommandGroup(
        Cmds.setState(shooterState),
        new ParallelRaceGroup(new WaitUntilCommand(() -> Robot.shooter.isAtTarget()), new WaitCommand(3)),
        Cmds.setState(Intake.State.INTAKE_GP),
        Cmds.setState(FeederState.FEED_SHOT),
        new ParallelRaceGroup(
            Commands.sequence(
                new WaitCommand(1.5), new PrintCommand("Ending shot because static time passed")),
            Commands.sequence(
                new WaitUntilCommand(() -> !Robot.shooter.hasGamePiece()),
                new WaitCommand(0.15),
                new PrintCommand("Ending shot because dynamic time passed"))),
        Cmds.setState(FeederState.OFF));
  }

  public static Command runShooterPivot(ShooterPivot.State shooterPivotState) {
    return Cmds.setState(shooterPivotState);
  }
}
