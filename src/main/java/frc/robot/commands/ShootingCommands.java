package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
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
      Shooter.State shooterState,
      ShooterPivot.State shooterPivotState) {
    return new ParallelDeadlineGroup(
        ShootingCommands.runPathAndIntake(choreoPath),
        Commands.sequence(
            new WaitUntilCommand(() -> Robot.shooter.hasGamePiece()),
            ShootingCommands.runShooter(shooterState),
            new WaitCommand(0.1),
            ShootingCommands.runShooterPivot(shooterPivotState)));
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
        new WaitUntilCommand(() -> Robot.shooter.isAtTarget()),
        Cmds.setState(Intake.State.INTAKE_GP),
        Cmds.setState(FeederState.FEED_SHOT),
        new WaitCommand(0.25),
        Cmds.setState(ShooterState.OFF));
  }

  public static Command runShooterPivot(ShooterPivot.State shooterPivotState) {
    return Cmds.setState(shooterPivotState);
  }
}
