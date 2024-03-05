package frc.robot.util;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import lombok.NonNull;
import lombok.experimental.UtilityClass;
import org.littletonrobotics.junction.Logger;

@UtilityClass
public final class RedHawkUtil {

  /**
   * Checks whether the given REVLibError is actually an error, and then logs it to AdvantageScope
   * and SmartDasboard. SmartDashboard variable logged is "RevLibError" and "RevLibError/name"
   * AdvantageScope variable logged is "RevLibError/name"
   *
   * @param status A RevLibError
   * @param name The name of the RevLibError, logged (see description)
   */
  public static void errorHandleSparkMAX(@NonNull REVLibError status) {
    if (status != REVLibError.kOk) {
      StackTraceElement[] rawStackTrace = Thread.currentThread().getStackTrace();
      ErrHandler.getInstance()
          .addError(
              status.name()
                  + " StackTrace: "
                  + rawStackTrace[2].getFileName()
                  + ":"
                  + rawStackTrace[2].getLineNumber());
    }
  }

  public static void cOk(REVLibError status) {
    errorHandleSparkMAX(status);
  }

  public static Translation2d Pose2dToTranslation2d(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }

  public static Translation2d Translation3dTo2d(Translation3d trans) {
    return new Translation2d(trans.getX(), trans.getY());
  }

  /**
   * Checkes if given pose if past the mid point of the field form their community (exclusive).
   * Flips {@code pose} if on red alliance
   *
   * @param pose the pose to check
   */
  public static boolean pastMidPoint(Pose2d pose) {
    return Reflections.reflectIfRed(pose.getX()) > (FieldConstants.fieldLength / 2);
  }

  public static ChoreoTrajectory maybeFlip(ChoreoTrajectory traj) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return traj;
    }

    if (alliance.get() == Alliance.Blue) {
      return traj;
    } else {
      return traj.flipped();
    }
  }

  // public static PathPoint currentPositionPathPoint(Rotation2d heading) {
  // return new
  // PathPoint(RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getUsablePose()),
  // heading, Robot.swerveDrive.getUsablePose().getRotation(),
  // Robot.swerveDrive.getAverageVelocity());
  // }

  public static class ErrHandler {
    private static ErrHandler INSTANCE;

    /**
     * Gets the instance of the ErrHandler singleton
     *
     * @return The one instance of ErrHandler
     */
    public static ErrHandler getInstance() {
      if (INSTANCE == null) {
        INSTANCE = new ErrHandler();
      }
      return INSTANCE;
    }

    private ErrHandler() {}

    private List<String> errors = new ArrayList<>();

    /**
     * Adds an error to the list of errors. Must be called on the singleton (see {@code
     * getInstance}). Also logs.
     */
    public void addError(@NonNull String error) {
      this.errors.add(error);
      this.log();
    }

    public void log() {
      Logger.recordOutput("Errors", String.join(" \\\\ ", errors));
    }
  }

  public static Twist2d poseLog(final Pose2d transform) {
    final double kEps = 1E-9;

    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = transform.getRotation().getCos() - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  public static class Reflections {
    public static Translation2d reflectIfRed(Translation2d old) {
      var maybeAlliance = DriverStation.getAlliance();
      if (maybeAlliance.isPresent() && maybeAlliance.get() == Alliance.Red) {
        return reflect(old);
      }
      return old;
    }

    public static Translation3d reflectIfRed(Translation3d old) {
      var maybeAlliance = DriverStation.getAlliance();
      if (maybeAlliance.isPresent() && maybeAlliance.get() == Alliance.Red) {
        return reflect(old);
      }
      return old;
    }

    public static Translation2d reflectIfBlue(Translation2d old) {
      var maybeAlliance = DriverStation.getAlliance();
      if (maybeAlliance.isPresent() && maybeAlliance.get() == Alliance.Blue) {
        return reflect(old);
      }
      return old;
    }

    public static Translation2d reflect(Translation2d old) {
      return new Translation2d(FieldConstants.fieldLength - old.getX(), old.getY());
    }

    public static Translation3d reflect(Translation3d old) {
      return new Translation3d(FieldConstants.fieldLength - old.getX(), old.getY(), old.getZ());
    }

    public static double reflectIfRed(double x) {
      return reflectIfRed(new Translation2d(x, 0)).getX();
    }

    public static Rotation2d reflectIfRed(Rotation2d old) {
      var maybeAlliance = DriverStation.getAlliance();
      if (maybeAlliance.isPresent() && maybeAlliance.get() == Alliance.Red) {
        return old.minus(Rotation2d.fromDegrees(180));
      }
      return old;
    }

    public static Rotation2d reflectIfBlue(Rotation2d old) {
      var maybeAlliance = DriverStation.getAlliance();
      if (maybeAlliance.isPresent() && maybeAlliance.get() == Alliance.Blue) {
        return old.minus(Rotation2d.fromDegrees(180)).times(-1);
      }
      return old;
    }

    public static Pose2d reflectIfRed(Pose2d old) {
      return new Pose2d(reflectIfRed(old.getTranslation()), reflectIfRed(old.getRotation()));
    }

    public static Pose2d reflectIfBlue(Pose2d old) {
      return new Pose2d(reflectIfBlue(old.getTranslation()), reflectIfBlue(old.getRotation()));
    }
  }

  /**
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   */
  public static void configureCANSparkMAXStatusFrames(
      HashMap<PeriodicFrame, Integer> config, CANSparkBase... sparks) {
    config.forEach(
        (frame, ms) -> {
          for (CANSparkBase spark : sparks) {
            cOk(spark.setPeriodicFramePeriod(frame, ms));
          }
        });
  }

  public static void configurePigeonStatusFrames(
      Pigeon2 pigeon, HashMap<PigeonIMU_StatusFrame, Integer> config) {
    config.forEach(
        (frame, ms) -> {
          // pigeon.setStatusFramePeriod(frame, ms);
        });
  }

  public static double lerp(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }

  public static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
    return startValue.plus((endValue.minus(startValue)).times(t));
  }

  public static ChassisSpeeds lerp(ChassisSpeeds startValue, ChassisSpeeds endValue, double t) {
    return new ChassisSpeeds(
        lerp(startValue.vxMetersPerSecond, endValue.vxMetersPerSecond, t),
        lerp(startValue.vyMetersPerSecond, endValue.vyMetersPerSecond, t),
        lerp(startValue.omegaRadiansPerSecond, endValue.omegaRadiansPerSecond, t));
  }

  public static double getDistPerPulse(double diametre) {
    return (1.0 / Constants.DriveConstants.GEAR_RATIO) * Units.inchesToMeters(diametre) * Math.PI;
  }

  public static SwerveModulePosition moduleDelta(
      SwerveModulePosition before, SwerveModulePosition after) {
    return new SwerveModulePosition(after.distanceMeters - before.distanceMeters, after.angle);
  }

  public static void applyConfigs(TalonFX fx, TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      status = fx.getConfigurator().apply(config);
      if (status.isOK()) break;
    }

    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public static String getLogDirectory() {
    Date now = Date.from(Instant.now());
    String dateFormat = new SimpleDateFormat("MM/dd").format(now);
    return String.format("/U/%s/", dateFormat);
  }

  public static Command logShot() {
    return new InstantCommand(
        () -> {
          var pos = RedHawkUtil.Reflections.reflectIfBlue(Robot.swerveDrive.getUsablePose());
          var deg = pos.getRotation().getDegrees();
          deg = Math.signum(deg) == -1 ? deg + 360 : deg;
          Logger.recordOutput(
              "/Shot log",
              String.format(
                  "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                  DriverStation.getMatchTime(),
                  deg,
                  ((Robot.shooter.inputs.leftSpeedRPM + Robot.shooter.inputs.rightSpeedRPM) / 2),
                  Robot.shooterPivot.getCurrentAngle(),
                  Robot.elevator.getCurrentHeight(),
                  pos.getTranslation().getX(),
                  pos.getTranslation().getY(),
                  // TODO: SHOULD CHASSIS SPEEDS BE FLIPPED?
                  Robot.swerveDrive.getChassisSpeeds().vxMetersPerSecond,
                  Robot.swerveDrive.getChassisSpeeds().vyMetersPerSecond,
                  0.5));
        });
  }

  public static void logShotFirst() {
    Logger.recordOutput(
        "/Shot log", "time,theta,shooter_speed,pivot_angle,elevator_height,x,y,vx,vy,went_in");
  }
}
