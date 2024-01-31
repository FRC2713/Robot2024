package frc.robot.commands.otf;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SwerveHeadingController;
import org.littletonrobotics.junction.Logger;

public class RotateScore extends SequentialCommandGroup {
  public static Rotation2d getOptimalAngle(Pose2d position) {
    var speakerLoc = RedHawkUtil.Reflections.reflectIfRed(new Translation2d(0.695, 5.552));
    var distance = position.getTranslation().getDistance(speakerLoc);

    var optimalAngle = Math.acos((position.getX() - speakerLoc.getX()) / distance);
    if (position.getY() < speakerLoc.getY()) {
      optimalAngle *= -1;
    }
    Logger.recordOutput(
        "Optimal Angle", new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return RedHawkUtil.Reflections.reflectIfRed(new Rotation2d(optimalAngle));
  }

  public static Command goOptimalAngle() {
    return new InstantCommand(
        () -> {
          Rotation2d optimal = RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());
          SwerveHeadingController.getInstance().setSetpoint(optimal);
        });
  }
}
