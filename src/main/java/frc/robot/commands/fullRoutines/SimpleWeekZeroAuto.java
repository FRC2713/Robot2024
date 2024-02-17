package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;

public class SimpleWeekZeroAuto extends SequentialCommandGroup {
  public SimpleWeekZeroAuto() {
    addCommands(
        Shooter.Commands.setMotionMode(Shooter.MotionMode.FENDER_SHOT_CLOSED_LOOP),
        // new WaitUntilCommand(Robot.shooter::isAtTarget),
        Feeder.Commands.setMotionMode(Feeder.MotionMode.SEND_TO_SHOOTER),
        Intake.Commands.setMotionMode(Intake.MotionMode.SEND_GP_TO_FEEDER),
        new WaitCommand(3),
        Feeder.Commands.setMotionMode(Feeder.MotionMode.OFF),
        Shooter.Commands.setMotionMode(Shooter.MotionMode.OFF),
        Intake.Commands.setMotionMode(Intake.MotionMode.OFF));
  }
}
