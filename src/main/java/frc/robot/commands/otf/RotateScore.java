package frc.robot.commands.otf;

import static frc.robot.util.RedHawkUtil.Translation3dTo2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class RotateScore extends SequentialCommandGroup {
  private static Translation3d speakerLoc;
  private static Translation3d speakerLocPivot;
  private static Translation3d ampLoc;

  static {
    updateSpeakerLoc();
    updateAmpLoc();
  }

  public static void updateSpeakerLoc() {
    speakerLoc =
        RedHawkUtil.Reflections.reflectIfRed(
            new Translation3d(0.695 - Units.inchesToMeters(18 + 6), 5.552, 2.11));
    speakerLocPivot =
        RedHawkUtil.Reflections.reflectIfRed(
            new Translation3d(0.695 - Units.inchesToMeters(18), 5.552, 2.11));
  }

  public static void updateAmpLoc() {
    ampLoc = RedHawkUtil.Reflections.reflectIfRed(new Translation3d(2, 6, 0));
    Logger.recordOutput("Field/amp_loc", new Translation2d(ampLoc.getX(), ampLoc.getY()));
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
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLocPivot));
    Logger.recordOutput("OTF/Speaker Distance", distance);
    Logger.recordOutput("OTF/Optimal Pivot Angle", Angle.get(distance));
    return MathUtil.clamp(Angle.get(distance), 0, 54);
  }

  public static double getOptimalShooterSpeed(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLocPivot));
    double shooterSpeed = shooterNominalSpeed.get(distance);
    Logger.recordOutput("OTF/Speaker Distance", distance);
    Logger.recordOutput("OTF/Shooter Nominal Speed", shooterSpeed);
    return shooterSpeed;
  }

  public static double getOptimalLobShotShooterSpeed(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(ampLoc));
    double shooterSpeed = lobShotShooterNominalSpeed.get(distance);

    Logger.recordOutput("OTF/Lob Shot Distance", distance);
    Logger.recordOutput("OTF/Lob Shot Speed Raw", shooterSpeed);

    double shooterSpeedVel =
        shooterSpeed
            - MathUtil.clamp(
                (Robot.swerveDrive.getChassisSpeeds().vxMetersPerSecond / 4) * 200, 0, 100);
    Logger.recordOutput("OTF/Lob Shot Speed Vel", shooterSpeedVel);

    return shooterSpeedVel;
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
          // From match WCMP
          put(2.11, 35.);
          // From some other source
          put(2.27, 32.);
          put(2.5, 32.);
          put(2.53, 30.);

          // From match WCMP
          put(2.9, 29.5);

          // From practice field DCMP
          put(3., 29.);
          // From practice field WCMP
          put(3.08, 29.);
          put(3.25, 24.);

          // From some other source
          // put(3.1, 27.);

          // From practice field WCMP
          put(3.78, 23.);
          put(3.9, 22.);
          put(4., 22.);
          put(4.3, 20.);
          // From practice field DCMP
          put(4.9, 19.5);
          // From practice field WCMP
          put(5.6, 19.);
          // From practice field DCMP
          put(5.9, 18.);

          // put(4., 20.);
          // Extrapolating with exponential regression
          // put(4.5, 20.);
          // put(5.0, 14.76);
          // put(5.5, 12.73);
          // put(6.0, 10.98);
          // put(6.5, 19.47);
        }
        /*
         *
         * 2.5,31
         * 3.25,24
         * 3.9,22
         * 4.3,19.5
         *
         *
         *
         * 4.0,24
         *
         */
      };

  private static InterpolatingTreeMap<Double, Double> shooterNominalSpeed =
      new InterpolatingTreeMap<>() {
        {
          put(1.0, 3500.);
          put(4.5, 4000.);
          put(5.0, 5000.);
          put(6.0, 5500.);
        }
      };

  private static InterpolatingTreeMap<Double, Double> lobShotShooterNominalSpeed =
      new InterpolatingTreeMap<>() {
        {
          put(5.406, 2200. - 50);
          put(6.5, 2400. - 50);
          put(7.433, 2500. - 50);
          put(8., 3000. - 50);
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
