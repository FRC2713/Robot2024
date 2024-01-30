package frc.robot.commands.otf;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.util.FieldConstants;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SwerveHeadingController;
import org.littletonrobotics.junction.Logger;

public class RotateScore extends SequentialCommandGroup {
  public static double getOptimalAngle(Pose2d position) {
    var supposedSpeakerLoc = new Translation2d(0.695, 5.552);
    var speakerLoc = supposedSpeakerLoc;
    if (!RedHawkUtil.pastMidPoint(position)) {
      speakerLoc =
          new Translation2d(
              FieldConstants.fieldLength - supposedSpeakerLoc.getX(), supposedSpeakerLoc.getY());
    }
    var distance = position.getTranslation().getDistance(speakerLoc);

    var optimalAngle = Math.acos((position.getX() - speakerLoc.getX()) / distance);
    if (position.getY() < speakerLoc.getY()) {
      optimalAngle *= -1;
    }
    Logger.recordOutput(
        "Optimal Angle", new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return optimalAngle;
  }

  public static Command goOptimalAngle() {
    return new InstantCommand(
        () -> {
          double optimal = RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());
          SwerveHeadingController.getInstance().setSetpoint(new Rotation2d(optimal));
        });
  }
}
