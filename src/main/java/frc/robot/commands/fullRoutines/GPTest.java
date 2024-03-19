package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.VehicleState;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;

public class GPTest extends RHRFullRoutine {
  public GPTest() {
    traj1 = Choreo.getTrajectory("Bottom Two.1");

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),
        new InstantCommand(
            () -> {
              VehicleState.getInstance().resetClosestGP();
              Robot.swerveDrive.setMotionMode(MotionMode.DRIVE_TOWARDS_GP);
            }));
  }
}
