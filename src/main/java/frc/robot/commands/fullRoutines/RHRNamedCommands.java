package frc.robot.commands.fullRoutines;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class RHRNamedCommands {
  public static void registerGenericCommands() {
    ArrayList<Pair<String, Command>> list = new ArrayList<Pair<String, Command>>();
    list.add(
        new Pair<String, Command>(
            "FireOuttake", new InstantCommand(() -> Logger.recordOutput("Firing", true))));
    list.add(
        new Pair<String, Command>(
            "StopFiringOuttake", new InstantCommand(() -> Logger.recordOutput("Firing", false))));

    NamedCommands.registerCommands(list);
  }
}
