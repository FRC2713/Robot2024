package frc.robot.commands.fullRoutines;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class ThreePieceChoreo {

  public static Command choreoCommandBuilder(ChoreoTrajectory traj) {
    var alliance = DriverStation.getAlliance();
    boolean useAllianceColour = false;
    if (alliance.isPresent()) {
      useAllianceColour = alliance.get() == DriverStation.Alliance.Red;
    }

    return Choreo.choreoSwerveCommand(
        traj, //
        Robot.swerveDrive::getUsablePose, //
        new PIDController(7, 0.0, 0.0), //
        new PIDController(7, 0.0, 0.0), //
        new PIDController(7, 0.0, 0.0), //
        (ChassisSpeeds speeds) -> {
          Robot.swerveDrive.setDesiredChassisSpeeds(speeds);
        },
        useAllianceColour,
        Robot.swerveDrive //
        );
  }

  public static Command getAutonomousCommand() {
    // PathPlannerPath p = PathPlannerPath.fromChoreoTrajectory("3 Piece Choreo");
    // return new SequentialCommandGroup(
    //     new InstantCommand(
    //         () -> {
    //           Robot.swerveDrive.resetOdometry(p.getPreviewStartingHolonomicPose());
    //         }),
    //     new RHRPathPlannerAuto("3 Piece Choreo"));

    ChoreoTrajectory firstTraj = Choreo.getTrajectory("3 Piece Choreo.1"); //
    ChoreoTrajectory secondTraj = Choreo.getTrajectory("3 Piece Choreo.2");
    ChoreoTrajectory thirdTraj = Choreo.getTrajectory("3 Piece Choreo.3");
    //
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(firstTraj.getInitialPose());
            }),
        choreoCommandBuilder(firstTraj),
        new WaitCommand(1),
        choreoCommandBuilder(secondTraj),
        new WaitCommand(1),
        choreoCommandBuilder(thirdTraj));
  }
}
