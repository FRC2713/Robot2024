package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;

public class Cmds {
  public static Command setState(MotionMode state) {
    return Commands.runOnce(() -> Robot.swerveDrive.setMotionMode(state));
  }

  public static Command setState(Elevator.State state) {
    return Commands.runOnce(() -> Robot.elevator.setState(state));
  }

  public static Command setState(Intake.State state) {
    return Commands.runOnce(() -> Robot.intake.setState(state));
  }

  public static Command setState(Shooter.State state) {
    return Commands.runOnce(() -> Robot.shooter.setState(state));
  }

  public static Command setState(ShooterPivot.State state) {
    return Commands.runOnce(() -> Robot.shooterPivot.setState(state));
  }
}
