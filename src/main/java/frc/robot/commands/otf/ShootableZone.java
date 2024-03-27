package frc.robot.commands.otf;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Robot;
import frc.robot.rhr.auto.RHRTrajectory;
import frc.robot.rhr.auto.RHRTrajectoryController;
import frc.robot.rhr.auto.RHRTrajectoryState;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.util.MotionHandler;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;
import org.opencv.core.Rect2d;

public class ShootableZone {
  public ShootableZone() {}

  public static Point pose2dToPoint(Pose2d pose) {
    return new Point(pose.getX(), pose.getY());
  }

  public static boolean isInShootableZone(Pose2d pose) {
    Rect2d zone = new Rect2d(new Point(), new Point(10, 10));
    return zone.contains(pose2dToPoint(pose));
  }

  public static double getIdealRotation(Pose2d pose) {
    Pose2d poseOfTarget = new Pose2d(new Translation2d(0, 5.531), new Rotation2d());
    double poseX = pose.getX();

    // Adjacent over Hypotenuse
    double angle =
        Math.acos(poseX / poseOfTarget.getTranslation().getDistance(pose.getTranslation()));

    if (pose.getY() < poseOfTarget.getY()) {
      angle = -angle;
    }

    return angle;
  }

  public Command run() {
    Pose2d pose = Robot.swerveDrive.getEstimatedPose();

    boolean shouldShoot = isInShootableZone(pose);
    Logger.recordOutput("AutoFiring/ShouldShoot", shouldShoot);
    // if (shouldShoot) {
    return new InstantCommand(
        () -> {
          double angle = getIdealRotation(Robot.swerveDrive.getEstimatedPose());

          Logger.recordOutput("AutoFiring/Angle", angle);

          Robot.swerveDrive.setMotionMode(MotionMode.TRAJECTORY);

          List<RHRTrajectoryState> states = new ArrayList<>();
          states.add(
              new RHRTrajectoryState(0, Robot.swerveDrive.getEstimatedPose(), new ChassisSpeeds()));
          states.add(
              new RHRTrajectoryState(
                  2,
                  new Pose2d(
                      Robot.swerveDrive.getEstimatedPose().getTranslation(),
                      Rotation2d.fromRadians(angle)),
                  new ChassisSpeeds()));
          RHRTrajectoryController.getInstance().setTrajectory(new RHRTrajectory(states));
          new RepeatCommand(
                  new InstantCommand(
                      () -> {
                        Robot.swerveDrive.setDesiredChassisSpeeds(
                            MotionHandler.driveTrajectory(Robot.swerveDrive.getEstimatedPose()));
                      }))
              .schedule();
        });
    // }
    // return new InstantCommand();
  }
}
