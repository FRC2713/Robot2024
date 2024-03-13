package frc.robot.commands;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
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

  public static Command runPathAndShoot(
      ChoreoTrajectory choreoPath,
      Shooter.FeederState shooterState,
      ShooterPivot.State shooterPivotState) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            ShootingCommands.runShooterPivot(shooterPivotState),
            ShootingCommands.runPath(choreoPath)),
        ShootingCommands.runShooter(shooterState));
  }

  public static Command runShooterAndPivot(
      Shooter.FeederState shooterState, ShooterPivot.State shooterPivotState) {
    return new SequentialCommandGroup(
        ShootingCommands.runShooterPivot(shooterPivotState),
        ShootingCommands.runShooter(shooterState));
  }

  public static Command runIntake() {
    return Commands.sequence(
        Intake.Commands.setMotionMode(Intake.State.INTAKE_GP),
        Shooter.Commands.setFeederState(Shooter.FeederState.INTAKING),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING));
  }

  public static Command runShooter(Shooter.FeederState shooterState) {
    return new SequentialCommandGroup(
        Shooter.Commands.setFeederState(shooterState),
        new WaitUntilCommand(() -> Robot.shooter.isAtTarget()),
        Intake.Commands.setMotionMode(Intake.State.INTAKE_GP),
        new WaitCommand(0.25),
        Shooter.Commands.setFeederState(Shooter.FeederState.OFF));
  }

  public static Command runShooterPivot(ShooterPivot.State shooterPivotState) {
    return ShooterPivot.Commands.setMotionMode(shooterPivotState);
  }
}
