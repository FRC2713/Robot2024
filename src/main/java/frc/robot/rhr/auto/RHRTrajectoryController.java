package frc.robot.rhr.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class RHRTrajectoryController {
  private static RHRTrajectoryController instance;

  private RHRTrajectoryController() {
    rController.enableContinuousInput(-180, 180);
  }

  public static RHRTrajectoryController getInstance() {
    if (instance == null) {
      instance = new RHRTrajectoryController();
    }

    return instance;
  }

  private RHRTrajectory trajectory;
  private Timer timer = new Timer();
  private PIDController xController = new PIDController(0, 0, 0),
      yController = new PIDController(0, 0, 0),
      rController = new PIDController(0, 0, 0);

  public void setTrajectory(RHRTrajectory trajectory) {
    this.trajectory = trajectory;
  }

  public boolean isFinished(Pose2d currentPose) {
    var sample = trajectory.sample(timer.get());
    var error = sample.pose().relativeTo(currentPose);
    return timer.hasElapsed(trajectory.getDuration())
        && error.getX() <= 0.1
        && error.getY() <= 0.1
        && error.getRotation().getDegrees() <= 0.1;
  }

  public ChassisSpeeds calculate(Pose2d currentPose) {
    if (trajectory == null) {
      return new ChassisSpeeds();
    }

    if (timer.get() == 0) {
      timer.start();
    }

    var sample = trajectory.sample(timer.get());
    var error = sample.pose().relativeTo(currentPose);

    double xFF = sample.chassisSpeeds().vxMetersPerSecond;
    double yFF = sample.chassisSpeeds().vyMetersPerSecond;
    double rFF = sample.chassisSpeeds().omegaRadiansPerSecond;

    double xFB = xController.calculate(currentPose.getX(), sample.pose().getX());
    double yFB = yController.calculate(currentPose.getY(), sample.pose().getY());
    double rFB =
        rController.calculate(
            currentPose.getRotation().getDegrees(), sample.pose().getRotation().getDegrees());

    if (isFinished(currentPose)) {
      return new ChassisSpeeds();
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFB, yFF + yFB, rFF + rFB, currentPose.getRotation());
    }
  }
}
