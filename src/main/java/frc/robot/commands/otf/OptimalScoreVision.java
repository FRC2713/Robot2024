package frc.robot.commands.otf;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OptimalScoreVision extends SequentialCommandGroup {

  public OptimalScoreVision() {
    // Commands go here
  }

  public double getHeadingAngle() {
    // Impilment method to pick which april tag is used to calculate heading change
    return 1;
  }

  public double getPivotAngle() {
    // Impliment method to calculate pivot angle
    return 1;
  }
}
