package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
  private final ShooterPivotIO IO;

  public ShooterPivot(ShooterPivotIO IO) {
    this.IO = IO;
  }
}
