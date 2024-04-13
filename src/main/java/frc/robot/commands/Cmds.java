package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.VehicleState;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter.FeederState;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;

public class Cmds {
  public static Command setState(MotionMode state) {
    if (state == MotionMode.ALIGN_TO_TAG) {
      return Commands.sequence(
          new InstantCommand(() -> VehicleState.getInstance().setRunningAlignToTag(false)),
          Commands.runOnce(() -> Robot.swerveDrive.setMotionMode(state)));
    }
    return Commands.runOnce(() -> Robot.swerveDrive.setMotionMode(state));
  }

  public static Command setState(Elevator.State state) {
    return Commands.runOnce(() -> Robot.elevator.setState(state));
  }

  public static Command setState(Intake.State state) {
    return Commands.runOnce(() -> Robot.intake.setState(state));
  }

  public static Command setState(ShooterState state) {
    return Commands.runOnce(() -> Robot.shooter.setShooterState(state));
  }

  public static Command setState(FeederState state) {
    return Commands.runOnce(() -> Robot.shooter.setFeederState(state));
  }

  public static Command setState(ShooterPivot.State state) {
    return Commands.runOnce(() -> Robot.shooterPivot.setState(state));
  }
}
