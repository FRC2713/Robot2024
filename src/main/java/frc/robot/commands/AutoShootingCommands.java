package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class AutoShootingCommands {

  public static Command runPath(String choreoPath) {
    ChoreoTrajectory traj = Choreo.getTrajectory(choreoPath);
    return SwerveSubsystem.Commands.choreoCommandBuilder(traj);
  }

  public static Command runPathAndIntake(String choreoPath) {
    return new SequentialCommandGroup(
        AutoShootingCommands.runIntake(), AutoShootingCommands.runPath(choreoPath), new WaitCommand(1));
  }

  public static Command runPathAndShoot(
      String choreoPath, Shooter.State shooterState, ShooterPivot.State shooterPivotState) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            AutoShootingCommands.runShooterPivot(shooterPivotState),
            AutoShootingCommands.runPath(choreoPath)),
        AutoShootingCommands.runShooter(shooterState));
  }

  public static Command runIntake() {
    return new InstantCommand(() -> Robot.intake.state = Intake.State.INTAKE_GP);
  }

  public static Command runShooter(Shooter.State shooterState) {
    return new SequentialCommandGroup(
        Shooter.Commands.setState(shooterState),
        new WaitUntilCommand(() -> Robot.shooter.isAtTarget()),
        Intake.Commands.setMotionMode(Intake.State.INTAKE_GP),
        new WaitCommand(1),
        Shooter.Commands.setState(Shooter.State.OFF));
  }

  public static Command runShooterPivot(ShooterPivot.State shooterPivotState) {
    return ShooterPivot.Commands.setMotionMode(shooterPivotState);
  }
}
