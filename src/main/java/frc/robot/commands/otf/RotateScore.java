package frc.robot.commands.otf;

import static frc.robot.util.RedHawkUtil.Translation3dTo2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class RotateScore extends SequentialCommandGroup {
  private static Translation3d speakerLoc;

  static {
    updateSpeakerLoc();
  }

  public static void updateSpeakerLoc() {
    speakerLoc =
        RedHawkUtil.Reflections.reflectIfRed(
            new Translation3d(0.695 - Units.inchesToMeters(18), 5.552, 2.11));
  }

  public static Rotation2d getOptimalAngle(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));

    var optimalAngle = Math.acos((position.getX() - speakerLoc.getX()) / distance);
    if (position.getY() < speakerLoc.getY()) {
      optimalAngle *= -1;
    }
    Logger.recordOutput("OTF/Speaker Loc", new Pose3d(speakerLoc, new Rotation3d()));
    Logger.recordOutput(
        "OTF/Optimal Angle", new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return new Rotation2d(optimalAngle);
  }

  public static Double getOptimalShooterAngle(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));
    Logger.recordOutput(
        "OTF/Speaker Distance", distance);
    Logger.recordOutput("OTF/Optimal Pivot Angle", Angle.get(distance));
    return Angle.get(distance);
  }

  private static InterpolatingTreeMap<Double, Double> Angle =
      new InterpolatingTreeMap<>() {
        {
          // Dist (metres), Angle (Degrees)
          put(0., 0.);
          put(1., 5.);
          put(2., 10.);
          put(3., 15.);
          put(4., 20.);
        }
      };
}
