package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.commands.otf.RotateScore;
import frc.robot.rhr.auto.RHRTrajectoryController;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

public class MotionHandler {

  /**
   * Calculates SwerveModuleState objects using the heading controller.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveHeadingController() {
    double speedFactor = (Robot.driver.getLeftTriggerAxis() > 0.25) ? 0.33 : 1.0;

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
        ySpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
        Units.degreesToRadians(SwerveHeadingController.getInstance().update()) * speedFactor,
        Robot.swerveDrive.getYaw());
  }

  /**
   * Calculates SwerveModuleState objects using pure driver control.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveFullControl() {
    double speedFactor = 1; // Robot.driver.rightBumper().getAsBoolean() ? 0.33 : 1.0;

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double rSpeed = (-Robot.driver.getRightX()) * speedFactor;
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
        ySpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
        rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
        Robot.swerveDrive.getYaw());
  }

  /**
   * Calculates swerveModuleStates using the current trajectory.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveTrajectory(Pose2d currentPose) {
    return RHRTrajectoryController.getInstance().calculate(currentPose);
  }

  /**
   * Sets the robot to an unmoving lockdown configuration which is difficult to push.
   *
   * @return The lockdown array of swerveModuleStates.
   */
  public static SwerveModuleState[] lockdown() {
    SwerveModuleState[] swerveModuleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

    return swerveModuleStates;
  }

  public static ChassisSpeeds driveAlignToTag() {
    // if (VehicleState.getInstance().isShouldUpdateCenterTagAlignment()) {

    // var error = VehicleState.getInstance().getCenterTagError();

    // if (error.isPresent()) {
    //   SwerveHeadingController.getInstance()
    //       .setSetpoint(Robot.swerveDrive.getYaw().minus(error.get()));

    //   VehicleState.getInstance().setShouldUpdateCenterTagAlignment(false);
    // }
    // }

    SwerveHeadingController.getInstance()
        .setSetpoint(RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose()));

    //   VehicleState.getInstance().setShouldUpdateCenterTagAlignment(false);
    // }

    return driveHeadingController();
  }

  public static ChassisSpeeds goClosestGP(ObjectDetection gp) {
    Logger.recordOutput("OTF/DrivingToGP/Doing it", true);
    Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Everything good");

    var angle = 0;

    // Get robot angle from where centre point is
    if (gp.centre.x > 0.5) {
      angle += Constants.AutoIntakeConstants.MAX_TX_LL1 * (gp.centre.x - 0.5);
    }
    if (gp.centre.x < 0.5) {
      angle += Constants.AutoIntakeConstants.MIN_TX_LL1 * (gp.centre.x + 0.5);
    }

    Logger.recordOutput("OTF/DrivingToGP/Angle", angle);
    Logger.recordOutput("OTF/DrivingToGP/Closest X", gp.centre.x);

    SwerveHeadingController.getInstance()
        .setSetpoint(Rotation2d.fromDegrees(angle).plus(Robot.swerveDrive.getYaw()));

    return ChassisSpeeds.fromRobotRelativeSpeeds(0.5, 0, 0, Rotation2d.fromDegrees(0));
  }

  public static boolean hasGPLock = false;
  public static ObjectDetection closestResult = new ObjectDetection(new Point(), 0, 0);

  public static ChassisSpeeds driveTowardsGP() {
    Logger.recordOutput("OTF/DrivingToGP/HasGPLock", hasGPLock);

    if (Robot.shooter.hasGamePiece()) {
      Logger.recordOutput("OTF/DrivingToGP/Doing it", false);
      Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Has GP");
      return driveFullControl();
    }

    if (hasGPLock) {
      return goClosestGP(closestResult);
    }

    var results = getObjectDetectionResults();

    for (var result : results) {
      if (result.goodness() > closestResult.goodness()) {
        closestResult = result;
      }
    }

    Logger.recordOutput("OTF/DrivingToGP/Goodness", closestResult.goodness());

    if (closestResult.goodness() < 0.0) {
      Logger.recordOutput("OTF/DrivingToGP/Doing it", false);
      Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Goodness too low");
      return driveFullControl();
    }

    hasGPLock = true;

    return goClosestGP(closestResult);
  }

  private static ArrayList<ObjectDetection> getObjectDetectionResults() {
    return Robot.visionFront.detections;
  }
}
