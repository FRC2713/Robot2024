package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootingCommands {
  public static SequentialCommandGroup FeederShotCommands() {
    return new SequentialCommandGroup(
        // new WaitUntilCommand(() -> Robot.feeder.motionMode == Feeder.MotionMode.HOLD_GAMEPIECE),
        // ShooterPivot.Commands.setMotionMode(ShooterPivot.State.SHORT_AUTO_SHOTS));
        );
  }

  public static SequentialCommandGroup FullShotCommands() {
    return new SequentialCommandGroup(
        // new WaitUntilCommand(() -> Robot.intake.state == Intake.State.HOLDING_GP),
        // ShooterPivot.Commands.setMotionMode(ShooterPivot.State.FEED_CLOSED_LOOP),
        FeederShotCommands());
  }
}
