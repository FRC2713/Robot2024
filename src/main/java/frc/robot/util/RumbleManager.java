package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class RumbleManager {
  public RumbleManager() {}

  public static Command driverBigOneSec() {
    return Commands.sequence(setDriveRumble(1), new WaitCommand(1), setDriveRumble(0));
  }

  public static Command driverSmallOneSec() {
    return Commands.sequence(setDriveRumble(0.5), new WaitCommand(1), setDriveRumble(0));
  }

  public static Command setDriveRumble(double magnitude) {
    return new InstantCommand(
        () -> Robot.driver.getHID().setRumble(RumbleType.kBothRumble, magnitude));
  }
}
