package frc.robot.subsystems.visionIO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase {
  private Vision[] visions;

  public VisionManager(Vision... visions) {
    this.visions = visions;
  }

  public void periodic() {}
}
