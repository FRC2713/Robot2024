package frc.robot.subsystems.swerveIO;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerveIO.module.SwerveModule;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIO;
import frc.robot.util.MotionHandler;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY,
    LOCKDOWN,
  }

  SwerveIO io;
  public final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveOdometry odometry;
  private final SwerveDrivePoseEstimator poseEstimator;

  public static double allianceFlipper = 1;

  private MotionMode motionMode = MotionMode.FULL_DRIVE;
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          DriveConstants.FRONT_LEFT_LOCATION,
          DriveConstants.FRONT_RIGHT_LOCATION,
          DriveConstants.BACK_LEFT_LOCATION,
          DriveConstants.BACK_RIGHT_LOCATION);

  /**
   * Creates a new SwerveSubsystem (swerve drive) object.
   *
   * @param swerveIO The IO layer of the swerve drive. Change this to change which gyro you're using
   *     (SwerveModuleIOPigeon2 vs SwerveModuleIOSim)
   * @param frontLeft The IO layer for the front left swerve module. Change this to change which
   *     motor controller you're using (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param frontRight The IO layer for the front right swerve module.
   * @param backLeft The IO layer for the back left swerve module.
   * @param backRight The IO layer for the back left swerve module.
   */
  public SwerveSubsystem(
      SwerveIO swerveIO,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    this.frontLeft = new SwerveModule(frontLeft, Constants.DriveConstants.FRONT_LEFT);
    this.frontRight = new SwerveModule(frontRight, Constants.DriveConstants.FRONT_RIGHT);
    this.backLeft = new SwerveModule(backLeft, Constants.DriveConstants.BACK_LEFT);
    this.backRight = new SwerveModule(backRight, Constants.DriveConstants.BACK_RIGHT);
    io = swerveIO;
    io.updateInputs(inputs, kinematics, getModulePositions());

    odometry =
        new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(inputs.gyroYawPosition),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.backLeft.getPosition(),
              this.backRight.getPosition()
            },
            new Pose2d());

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(inputs.gyroYawPosition),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.backLeft.getPosition(),
              this.backRight.getPosition()
            },
            new Pose2d(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
            new MatBuilder<>(Nat.N3(), Nat.N1())
                .fill(
                    Constants.LimeLightConstants.VISION_STD_DEVI_POSITION_IN_METERS,
                    Constants.LimeLightConstants.VISION_STD_DEVI_POSITION_IN_METERS,
                    Constants.LimeLightConstants.VISION_STD_DEVI_ROTATION_IN_RADIANS));
  }

  public void zeroGyro() {
    io.zeroGyro();
  }

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation The rotation to reset the gyro to.
   */
  public void resetGyro(Rotation2d rotation) {
    io.resetGyro(rotation);
  }

  /**
   * Resets the SwerveDriveOdometry to the given pose.
   *
   * @param pose The desired pose.
   */
  public void resetOdometry(Pose2d pose) {
    Logger.getInstance().recordOutput("Reset odometry to ", pose);
    odometry.resetPosition(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);

    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.backLeft.getPosition(),
          this.backRight.getPosition()
        },
        pose);
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return The position of the robot on the field.
   */
  private Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getUsablePose() {
    if (Constants.ENABLE_VISION_POSE_ESTIMATION) {
      return getEstimatedPose();
    } else {
      return getRegularPose();
    }
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(inputs.gyroYawPosition);
  }

  private Pose2d getRegularPose() {
    return odometry.getPoseMeters();
  }

  public double getTotalCurrentDraw() {
    return frontLeft.getTotalCurrentDraw()
        + frontRight.getTotalCurrentDraw()
        + backLeft.getTotalCurrentDraw()
        + backRight.getTotalCurrentDraw();
  }

  public void updateVisionPose(
      TimestampedDoubleArray fieldPoseArray, TimestampedDoubleArray cameraPoseArray) {
    double[] fVal = fieldPoseArray.value;
    double[] cVal = cameraPoseArray.value;
    double distCamToTag = Units.metersToInches(Math.abs(cVal[2]));

    Logger.getInstance().recordOutput("Vision/distCamToTag", distCamToTag);
    Pose2d fPose = new Pose2d(fVal[0], fVal[1], Rotation2d.fromDegrees(fVal[5]));

    if (fPose.getX() == 0 && fPose.getY() == 0 && fPose.getRotation().getDegrees() == 0) {
      Logger.getInstance().recordOutput("Vision/Got empty field pose", true);
      return;
    }
    Logger.getInstance().recordOutput("Vision/Got empty field pose", false);

    double jump_distance =
        Units.metersToInches(
            poseEstimator
                .getEstimatedPosition()
                .getTranslation()
                .getDistance(fPose.getTranslation()));
    Logger.getInstance().recordOutput("Vision/jump_distance", jump_distance);
    if (distCamToTag < Constants.LimeLightConstants.CAMERA_TO_TAG_MAX_DIST_INCHES
        && ((!DriverStation.isEnabled())
            || jump_distance < Constants.LimeLightConstants.MAX_POSE_JUMP_IN_INCHES)) {
      poseEstimator.addVisionMeasurement(fPose, Timer.getFPGATimestamp() - (fVal[6] / 1000.0));
    }
  }

  /**
   * Sets the desired states of the swerve modules.
   *
   * @param swerveModuleStates The array of desired swerveModuleStates. Ensure they are ordered the
   *     same way in this array as they are instantiated into SwerveDriveKinematics.
   */
  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
    this.desiredSpeeds = speeds;
  }

  public boolean gyroPitchHasChanged() {
    return inputs.gyroPitchPosition == inputs.previousgyroPitchPosition;
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition(),
    };
  }

  /**
   * Returns the average velocity of the swerve modules.
   *
   * @return The average velocity at which all the swerve modules are moving.
   */
  public double getAverageVelocity() {
    return (frontLeft.getMeasuredState().speedMetersPerSecond
            + frontRight.getMeasuredState().speedMetersPerSecond
            + backLeft.getMeasuredState().speedMetersPerSecond
            + backRight.getMeasuredState().speedMetersPerSecond)
        / 4;
  }

  // Only used for characterization
  public void applyVoltageForCharacterization(double volts) {
    frontLeft.applyVoltageForCharacterization(volts);
    frontRight.applyVoltageForCharacterization(volts);
    backLeft.applyVoltageForCharacterization(volts);
    backRight.applyVoltageForCharacterization(volts);
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {
    odometry.update(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  public void setMotionMode(MotionMode motionMode) {
    this.motionMode = motionMode;
  }

  public void seed() {
    frontLeft.seed();
    frontRight.seed();
    backLeft.seed();
    backRight.seed();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs, kinematics, getModulePositions());
    Logger.getInstance().processInputs("Swerve/Chassis", inputs);
    updateOdometry();

    switch (motionMode) {
      case FULL_DRIVE:
        setDesiredChassisSpeeds(MotionHandler.driveFullControl());
        break;
      case HEADING_CONTROLLER:
        setDesiredChassisSpeeds(MotionHandler.driveHeadingController());
        break;
      case LOCKDOWN:
        setModuleStates(MotionHandler.lockdown());
        break;
      case TRAJECTORY:
        setDesiredChassisSpeeds(MotionHandler.driveTrajectory(getUsablePose()));
        break;
      default:
        break;
    }

    if (motionMode != MotionMode.LOCKDOWN) {
      var moduleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          moduleStates,
          DriverStation.isAutonomous()
              ? DriveConstants.MAX_SWERVE_VEL_AUTO
              : DriveConstants.MAX_SWERVE_VEL);
      Logger.getInstance().recordOutput("Swerve/Desired Module States", moduleStates);
      setModuleStates(moduleStates);
    }

    Logger.getInstance().recordOutput("Swerve/Odometry/Wheel-Based", getRegularPose());
    Logger.getInstance().recordOutput("Swerve/Odometry/KF-Based", getEstimatedPose());
    Logger.getInstance().recordOutput("Swerve/Odometry/Useable", getUsablePose());
    Logger.getInstance()
        .recordOutput("Swerve/Desired speeds/x-mps", desiredSpeeds.vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Swerve/Desired speeds/y-mps", desiredSpeeds.vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Swerve/Desired speeds/r-radps", desiredSpeeds.omegaRadiansPerSecond);
  }

  public static class Commands {}
}
