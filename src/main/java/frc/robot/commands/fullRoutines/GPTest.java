package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.util.MotionHandler;
import frc.robot.util.ObjectDetection;
import org.opencv.core.Point;

public class GPTest extends RHRFullRoutine {
  public GPTest() {
    traj1 = Choreo.getTrajectory("Bottom Two.1");

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),
        new InstantCommand(
            () -> {
              MotionHandler.hasGPLock = false;
              MotionHandler.closestResult = new ObjectDetection(new Point(), 0, 0);
              Robot.swerveDrive.setMotionMode(MotionMode.DRIVE_TOWARDS_GP);
            }));
  }
}
