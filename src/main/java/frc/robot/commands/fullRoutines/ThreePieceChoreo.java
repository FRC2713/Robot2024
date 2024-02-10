package frc.robot.commands.fullRoutines;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.ErrorTracker;
import frc.robot.util.PIDFFGains;
import org.littletonrobotics.junction.Logger;

public class ThreePieceChoreo extends SequentialCommandGroup {
  public ErrorTracker errorTracker;

  public ChoreoControlFunction modifiedChoreoSwerveController(
      PIDController xController, PIDController yController, PIDController rotationController) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    errorTracker =
        new ErrorTracker(
            10, PIDFFGains.fromPIDGains(xController), PIDFFGains.fromPIDGains(rotationController));
    return (pose, referenceState) -> {
      Logger.recordOutput(
          "Choreo/Target Pose",
          new Pose2d(
              new Translation2d(referenceState.x, referenceState.y),
              Rotation2d.fromRadians(referenceState.heading)));

      var error =
          new Pose2d(
              new Translation2d(referenceState.x - pose.getX(), referenceState.y - pose.getY()),
              Rotation2d.fromRadians(referenceState.heading - pose.getRotation().getRadians()));

      errorTracker.addObservation(error);
      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF = referenceState.angularVelocity;

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
    };
  }

  public Command choreoCommandBuilder(ChoreoTrajectory traj) {
    var alliance = DriverStation.getAlliance();
    boolean useAllianceColour = false;
    if (alliance.isPresent()) {
      useAllianceColour = alliance.get() == DriverStation.Alliance.Red;
    }

    return new SequentialCommandGroup(
        Choreo.choreoSwerveCommand(
            traj, //
            Robot.swerveDrive::getUsablePose, //
            modifiedChoreoSwerveController(
                new PIDController(3, 0.0, 0.0), //
                new PIDController(3, 0.0, 0.0), //
                new PIDController(3, 0.0, 0.0)),
            (ChassisSpeeds speeds) -> {
              Robot.swerveDrive.setDesiredChassisSpeeds(speeds);
            },
            useAllianceColour,
            Robot.swerveDrive //
            ),
        new InstantCommand(() -> Robot.swerveDrive.setDesiredChassisSpeeds(new ChassisSpeeds())));
  }

  public ThreePieceChoreo() {
    ChoreoTrajectory firstTraj = Choreo.getTrajectory("3 Piece Choreo.1"); //
    ChoreoTrajectory secondTraj = Choreo.getTrajectory("3 Piece Choreo.2");
    ChoreoTrajectory thirdTraj = Choreo.getTrajectory("3 Piece Choreo.3");
    //
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(firstTraj.getInitialPose());
            }),
        ShootingCommands.FeederShotCommands(),
        new ParallelCommandGroup(
            choreoCommandBuilder(firstTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        ShootingCommands.FullShotCommands(),
        new ParallelCommandGroup(
            choreoCommandBuilder(secondTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        ShootingCommands.FullShotCommands(),
        new ParallelCommandGroup(
            choreoCommandBuilder(thirdTraj),
            Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP)),
        ShootingCommands.FullShotCommands(),
        new InstantCommand(
            () -> {
              errorTracker.printSummary("ThreePieceChoreo");
            }));
  }
}
