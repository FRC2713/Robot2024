package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

public class RumbleManager {
  private static RumbleManager instance;

  private Timer driverTimer, operatorTimer;
  private double driverDuration, operatorDuration;

  private RumbleManager() {
    driverTimer = new Timer();
    operatorTimer = new Timer();
  }

  public static RumbleManager getInstance() {
    if (instance == null) {
      instance = new RumbleManager();
    }

    return instance;
  }

  public void setDriver(double magnitude, double duration) {
    driverTimer.reset();
    driverTimer.start();
    this.driverDuration = duration;
    set(Robot.driver.getHID(), magnitude);
  }

  public void setDriverNoTimer(double magnitude) {
    set(Robot.driver.getHID(), magnitude);
  }

  public void stopDriver() {
    set(Robot.driver.getHID(), 0.0);
  }

  public void setOperator(double magnitude, double duration) {
    operatorTimer.reset();
    operatorTimer.start();
    this.operatorDuration = duration;
    set(Robot.operator.getHID(), magnitude);
  }

  public void set(XboxController hid, double magnitude) {
    hid.setRumble(RumbleType.kBothRumble, magnitude);
  }

  public void periodic() {
    if (driverTimer.hasElapsed(driverDuration)) {
      driverTimer.stop();
      set(Robot.driver.getHID(), 0);
    }
    if (operatorTimer.hasElapsed(operatorDuration)) {
      operatorTimer.stop();
      set(Robot.operator.getHID(), 0);
    }
  }
}
