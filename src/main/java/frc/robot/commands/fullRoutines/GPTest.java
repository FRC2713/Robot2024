package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.otf.OTFAmp;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class GPTest extends RHRFullRoutine {
  public GPTest() {
    traj1 = Choreo.getTrajectory("Bottom Two.1");

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1), OTFAmp.getInstance().runAndRegenerate());
  }
}
