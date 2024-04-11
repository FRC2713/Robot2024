package frc.robot.commands.otf;

import static frc.robot.util.RedHawkUtil.Translation3dTo2d;

import edu.wpi.first.math.MathUtil;
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
  private static Translation3d ampLoc;

  static {
    updateSpeakerLoc();
    updateAmpLoc();
  }

  public static void updateSpeakerLoc() {
    speakerLoc =
        RedHawkUtil.Reflections.reflectIfRed(
            new Translation3d(0.695 - Units.inchesToMeters(18), 5.552, 2.11));
  }

  public static void updateAmpLoc() {
    ampLoc = RedHawkUtil.Reflections.reflectIfRed(new Translation3d(2.5, 7.5, 0));
  }

  public static Rotation2d getOptimalAmpAngle(Pose2d position) {
    var x = position.getX() - ampLoc.getX();
    var y = position.getY() - ampLoc.getY();
    var optimalAngle = Math.atan2(y, x);
    Logger.recordOutput("OTF/Amp Loc", new Pose3d(ampLoc, new Rotation3d()));
    Logger.recordOutput(
        "OTF/Optimal Amp Angle",
        new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return new Rotation2d(optimalAngle);
  }

  public static Rotation2d getOptimalAngle(Pose2d position) {
    var x = position.getX() - speakerLoc.getX();
    var y = position.getY() - speakerLoc.getY();
    var optimalAngle = Math.atan2(y, x);
    Logger.recordOutput("OTF/Speaker Loc", new Pose3d(speakerLoc, new Rotation3d()));
    Logger.recordOutput(
        "OTF/Optimal Angle", new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return new Rotation2d(optimalAngle);
  }

  public static double getOptimalShooterAngle(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));
    Logger.recordOutput("OTF/Speaker Distance", distance);
    Logger.recordOutput("OTF/Optimal Pivot Angle", Angle.get(distance));
    return MathUtil.clamp(Angle.get(distance), 0, 54);
  }

  public static double getOptimalShooterSpeed(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));

    double shooterSpeed = shooterNominalSpeed.get(distance);

    Logger.recordOutput("OTF/Speaker Distance", distance);

    Logger.recordOutput("OTF/Shooter Nominal Speed", shooterSpeed);
    return shooterSpeed;
    // if (distance > 4.5) {
    //   return 5000;
    // } else {
    //   return 4000;
    // }
  }

  public static double getElevatorOptimalShooterAngle(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));
    Logger.recordOutput("OTF/Speaker Distance", distance);
    Logger.recordOutput("OTF/Elevator Optimal Pivot Angle", elevatorAngle.get(distance));
    return MathUtil.clamp(elevatorAngle.get(distance), 0, 54);
  }

  private static InterpolatingTreeMap<Double, Double> Angle =
      new InterpolatingTreeMap<>() {
        {
          // Dist (metres), Angle (Degrees)
          // From some other source
          put(1.08, 48.);
          put(1.31, 44.);
          put(1.62, 41.);
          put(1.955, 36.);
          put(2.27, 32.);
          put(2.5, 30.);
          put(2.53, 30.);

          // From practice field DCMP
          put(3., 28.);
          put(3.17, 26.);

          // From some other source
          // put(3.1, 27.);

          // From practice field DCMP
          put(3.825, 26.);
          put(4.15, 22.5);
          put(4.319, 21.);
          put(4.9, 19.5);

          // put(4., 20.);
          // Extrapolating with exponential regression
          // put(4.5, 20.);
          // put(5.0, 14.76);
          // put(5.5, 12.73);
          // put(6.0, 10.98);
          // put(6.5, 19.47);
        }
      };

  private static InterpolatingTreeMap<Double, Double> shooterNominalSpeed =
      new InterpolatingTreeMap<>() {
        {
          put(1.0, 3500.);
          put(4.5, 4000.);
          put(5.0, 5000.);
        }
      };

  private static InterpolatingTreeMap<Double, Double> elevatorAngle =
      new InterpolatingTreeMap<>() {
        {
          // Dist (metres), Angle (Degrees)
          put(1.08, 48. - 9);
          put(1.31, 44. - 9);
          put(1.62, 41. - 9);
          put(1.955, 36. - 9);
          put(4., 20. - 9);
          put(2.27, 32. - 9);
          put(2.5, 30. - 9);
          put(2.53, 30. - 9);
          put(3.1, 27. - 9);
        }
      };
}
