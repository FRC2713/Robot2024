package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MechanismManager {
  private final Mechanism2d mech;

  public MechanismManager() {
    mech = new Mechanism2d(50, 50);

    // Log to SmartDashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {}
}
