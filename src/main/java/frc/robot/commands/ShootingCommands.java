package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.subsystems.shooterPivot.ShooterPivot;

public class ShootingCommands {
  public static SequentialCommandGroup FeederShotCommands() {
    return new SequentialCommandGroup(
        new WaitUntilCommand(() -> Robot.feeder.motionMode == Feeder.MotionMode.HOLD_GAMEPIECE),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.MotionMode.SHORT_AUTO_SHOTS));
  }

  public static SequentialCommandGroup FullShotCommands() {
    return new SequentialCommandGroup(
        // new WaitUntilCommand(() -> Robot.intake.state == Intake.State.HOLDING_GP),
        ShooterPivot.Commands.setMotionMode(ShooterPivot.MotionMode.FEED_CLOSED_LOOP),
        FeederShotCommands());
  }
}
