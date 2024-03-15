package frc.robot.subsystems.swerveIO;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Robot;
import frc.robot.rhr.auto.RHRPathPlannerAuto;
import frc.robot.subsystems.swerveIO.module.SwerveModule;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIO;
import frc.robot.subsystems.visionIO.VisionIO.VisionInputs;
import frc.robot.subsystems.visionIO.VisionInfo;
import frc.robot.util.ErrorTracker;
import frc.robot.util.MotionHandler;
import frc.robot.util.PIDFFGains;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SwerveHeadingController;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY,
    LOCKDOWN,
    ALIGN_TO_TAG
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

  @Getter private MotionMode motionMode = MotionMode.FULL_DRIVE;
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          DriveConstants.FRONT_LEFT_LOCATION, DriveConstants.FRONT_RIGHT_LOCATION,
          DriveConstants.BACK_LEFT_LOCATION, DriveConstants.BACK_RIGHT_LOCATION);

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
            VecBuilder.fill(
                LimeLightConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev(),
                LimeLightConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev(),
                LimeLightConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev()),
            VecBuilder.fill(
                LimeLightConstants.POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS.translationalStDev(),
                LimeLightConstants.POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS.translationalStDev(),
                LimeLightConstants.POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS.rotationalStDev()));

    AutoBuilder.configureHolonomic(
        this::getUsablePose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (cs) -> {
          // this.setDesiredChassisSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(cs, getYaw()));
          this.setDesiredChassisSpeeds(cs);
        },
        new HolonomicPathFollowerConfig(
            Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_X.toPathplannerGains(),
            Constants.DriveConstants.Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION
                .toPathplannerGains(),
            4.5,
            0.4,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here

          Logger.recordOutput("PathPlanner/Current Pose", pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          Logger.recordOutput("PathPlanner/Target Pose", pose);
        });

    PathPlannerLogging.setLogActivePathCallback(
        path -> {
          Logger.recordOutput("PathPlanner/Active Path", path.toArray(Pose2d[]::new));
        });
  }

  public void zeroGyro() {
    io.zeroGyro();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getMeasuredState(),
      frontRight.getMeasuredState(),
      backLeft.getMeasuredState(),
      backRight.getMeasuredState()
    };
  }

  public double[] getAbsoluteEncoderAngles() {
    return new double[] {
      frontLeft.getAbsoluteEncoderAngle(),
      frontRight.getAbsoluteEncoderAngle(),
      backLeft.getAbsoluteEncoderAngle(),
      backRight.getAbsoluteEncoderAngle(),
    };
  }

  public void setDriveCurrentLimits(int amps) {
    for (var module : new SwerveModule[] {frontLeft, frontRight, backLeft, backRight}) {
      module.setDriveCurrentLimit(amps);
    }
  }

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation The rotation to reset the gyro to.
   */
  public void resetGyro(Rotation2d rotation) {
    System.err.println("Reset gyro!!!");
    io.resetGyro(rotation);
  }

  /**
   * Resets the SwerveDriveOdometry to the given pose.
   *
   * @param pose The desired pose.
   */
  public void resetOdometry(Pose2d pose) {
    Logger.recordOutput("Reset odometry to ", pose);

    // io.resetGyro(pose.getRotation());

    io.updateInputs(inputs, kinematics, getModulePositions());
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

  public ChassisSpeeds getChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(this.getRobotRelativeSpeeds(), this.getYaw());
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

  public void poseEstimationFromVision(VisionInputs left, VisionInputs right) {
    Pose2d visionBotPose = RedHawkUtil.Pose3dTo2d(left.botPoseBlue);
    // invalid LL data
    if (visionBotPose.getX() == 0.0) {
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference =
        getUsablePose().getTranslation().getDistance(visionBotPose.getTranslation());

    if (left.hasTarget) {
      double xyStds;
      double degStds;
      // multiple targets detected
      if (left.tagCount >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      // else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
      //   xyStds = 1.0;
      //   degStds = 12;
      // }
      // 1 target farther away and estimated pose is close
      // else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
      else {
        xyStds = 2.0;
        degStds = 30;
      }
      // }
      // conditions don't match to add a vision measurement
      // else {
      //   return;
      // }

      poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      poseEstimator.addVisionMeasurement(
          visionBotPose, Timer.getFPGATimestamp() - left.totalLatencyMs);
    }
  }

  public void updateOdometryFromVision(VisionInfo visionInfo, VisionInputs visionInputs) {
    if (!visionInputs.hasTarget) {
      return;
    }

    double jumpDistance =
        getUsablePose()
            .getTranslation()
            .getDistance(visionInputs.botPoseBlue.toPose2d().getTranslation());

    Logger.recordOutput("Vision/" + visionInfo.getNtTableName() + "/Jump Distance", jumpDistance);

    // Use the pose if
    //  - We are disabled, OR
    //  - We are within the jump distance
    boolean shouldUpdatePose =
        !DriverStation.isEnabled() || jumpDistance < LimeLightConstants.MAX_POSE_JUMP_METERS;
    Logger.recordOutput("Vision/Should update pose", shouldUpdatePose);
    if (shouldUpdatePose) {
      var stdevs =
          visionInputs.tagCount > 1
              ? LimeLightConstants.POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS
              : LimeLightConstants.POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS.multiplyByRange(1);

      poseEstimator.addVisionMeasurement(
          visionInputs.botPoseBlue.toPose2d(),
          visionInputs.botPoseBlueTimestamp,
          stdevs.toMatrix());
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
    Logger.recordOutput("Swerve/MotionMode", motionMode);
    io.updateInputs(inputs, kinematics, getModulePositions());
    Logger.processInputs("Swerve/Chassis", inputs);
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
        // setDesiredChassisSpeeds(MotionHandler.driveTrajectory(getUsablePose()));
        break;
      case ALIGN_TO_TAG:
        setDesiredChassisSpeeds(MotionHandler.driveAlignToTag());
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
      Logger.recordOutput("Swerve/Desired Module States", moduleStates);
      setModuleStates(moduleStates);
    }

    Logger.recordOutput("Swerve/Odometry/Wheel-Based", getRegularPose());
    Logger.recordOutput("Swerve/Odometry/KF-Based", getEstimatedPose());
    Logger.recordOutput("Swerve/Odometry/Useable", getUsablePose());
    Logger.recordOutput("Swerve/Desired speeds/x-mps", desiredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Swerve/Desired speeds/y-mps", desiredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Swerve/Desired speeds/r-radps", desiredSpeeds.omegaRadiansPerSecond);
  }

  public static class Commands {
    public static ErrorTracker errorTracker;

    public static ChoreoControlFunction modifiedChoreoSwerveController(
        PIDController xController, PIDController yController, PIDController rotationController) {
      rotationController.enableContinuousInput(-Math.PI, Math.PI);
      Commands.errorTracker =
          new ErrorTracker(
              10,
              PIDFFGains.fromPIDGains(xController),
              PIDFFGains.fromPIDGains(rotationController));
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

    public static Command setHeading(Rotation2d degrees) {
      return new InstantCommand(() -> SwerveHeadingController.getInstance().setSetpoint(degrees));
    }

    public static Command setHeadingandWait(Rotation2d degrees) {
      return new SequentialCommandGroup(
          setHeading(degrees),
          new WaitUntilCommand(SwerveHeadingController.getInstance()::atSetpoint));
    }

    public static Command getAutonomousCommand(String trajectory) {
      return new RHRPathPlannerAuto(trajectory);
    }

    public static Command resetOdometry(Pose2d pose) {
      return new InstantCommand(() -> Robot.swerveDrive.resetOdometry(pose));
    }

    public static Command resetGyro(ChoreoTrajectory traj) {
      return new InstantCommand(
          () -> Robot.swerveDrive.resetGyro(traj.getInitialPose().getRotation()));
    }

    public static Command resetOdometry(ChoreoTrajectory traj) {
      return new InstantCommand(() -> Robot.swerveDrive.resetOdometry(traj.getInitialPose()));
    }

    public static Command resetOdometryAndGyro(ChoreoTrajectory traj) {
      return new SequentialCommandGroup(resetGyro(traj), resetOdometry(traj));
    }

    public static Command resetOdometry(String trajectory) {
      PathPlannerPath p = PathPlannerPath.fromChoreoTrajectory(trajectory);
      return new InstantCommand(
          () -> Robot.swerveDrive.resetOdometry(p.getPreviewStartingHolonomicPose()));
    }

    public static Command choreoCommandBuilder(ChoreoTrajectory traj) {
      var alliance = DriverStation.getAlliance();
      boolean useAllianceColour = false;
      if (alliance.isPresent()) {
        useAllianceColour = alliance.get() == DriverStation.Alliance.Red;
      }

      return new SequentialCommandGroup(
          Choreo.choreoSwerveCommand(
              traj,
              Robot.swerveDrive::getUsablePose,
              modifiedChoreoSwerveController(
                  new PIDController(10, 0.0, 0.0),
                  new PIDController(10, 0.0, 0.0),
                  new PIDController(3, 0.0, 0.0)),
              (ChassisSpeeds speeds) -> {
                Robot.swerveDrive.setDesiredChassisSpeeds(speeds);
              },
              () -> false,
              Robot.swerveDrive //
              ),
          new InstantCommand(() -> Robot.swerveDrive.setDesiredChassisSpeeds(new ChassisSpeeds())));
    }
  }
}
