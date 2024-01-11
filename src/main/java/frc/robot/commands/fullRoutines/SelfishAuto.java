package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SelfishAuto {
  public static Command getAutonomousCommand() {
    return new SequentialCommandGroup(new PathPlannerAuto("Selfish Auto"));
  }
}
