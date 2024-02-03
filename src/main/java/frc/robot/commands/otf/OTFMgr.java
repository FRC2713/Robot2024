package frc.robot.commands.otf;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class OTFMgr {
  public Command runningCommand;

  public abstract Command run();

  public void cancelComand() {
    if (runningCommand != null) {
      runningCommand.cancel();
    }
  }
}
