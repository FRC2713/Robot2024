package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.shooterPivot.ShooterPivot;

public class ClimbingCommands {

  public static Command prepareForClimb() {
    return new ParallelCommandGroup(
        ShooterPivot.Commands.setMotionMode(ShooterPivot.State.PREPARE_FOR_CLIMB),
        Elevator.Commands.setState(Elevator.State.MAX_HEIGHT));
  }

  public static Command climb() {
    return Elevator.Commands.setState(Elevator.State.MIN_HEIGHT);
  }
}
