package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.VehicleState;
import frc.robot.commands.otf.RotateScore;
import frc.robot.rhr.auto.RHRTrajectoryController;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import org.littletonrobotics.junction.Logger;

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
        .setSetpoint(RotateScore.getOptimalAngle(Robot.swerveDrive.getEstimatedPose()));

    //   VehicleState.getInstance().setShouldUpdateCenterTagAlignment(false);
    // }

    return driveHeadingController();
  }

  public static ChassisSpeeds driveTowardsGP() {
    Logger.recordOutput("OTF/DrivingToGP/HasGPLock", VehicleState.getInstance().hasGPLock);

    if (Robot.shooter.hasGamePiece()) {
      Logger.recordOutput("OTF/DrivingToGP/Doing it", false);
      Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Has GP");
      return driveFullControl();
    }

    if (VehicleState.getInstance().hasGPLock) {
      return VehicleState.getInstance().goClosestGP();
    }

    var results = getObjectDetectionResults();

    for (var result : results) {
      if (result.goodness() > VehicleState.getInstance().closestResult.goodness()) {
        VehicleState.getInstance().closestResult = result;
      }
    }

    Logger.recordOutput(
        "OTF/DrivingToGP/Goodness", VehicleState.getInstance().closestResult.goodness());

    if (VehicleState.getInstance().closestResult.goodness() < 0.0000000000001) {
      Logger.recordOutput("OTF/DrivingToGP/Doing it", false);
      Logger.recordOutput("OTF/DrivingToGP/Reasoning", "Goodness too low");
      return driveFullControl();
    }

    VehicleState.getInstance().hasGPLock = true;
    VehicleState.getInstance().GPyaw = Robot.swerveDrive.getYaw();

    return VehicleState.getInstance().goClosestGP();
  }

  private static LimelightTarget_Detector[] getObjectDetectionResults() {
    return Robot.visionGP.detections;
  }
}
