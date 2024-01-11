package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class RHRNamedCommands {
  public static void registerGenericCommands() {
    ArrayList<Pair<String, Command>> list = new ArrayList<Pair<String, Command>>();
    list.add(
        new Pair<String, Command>(
            "Simple NamedCommand",
            new InstantCommand(
                () -> {
                  Logger.recordOutput("DidSOMETHING", false);
                })));
    list.add(new Pair<String, Command>("FireOuttake", new WaitCommand(2)));

    NamedCommands.registerCommands(list);
  }
}
