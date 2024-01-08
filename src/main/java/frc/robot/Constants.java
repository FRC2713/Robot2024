// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerveIO.module.ModuleInfo;
import frc.robot.subsystems.swerveIO.module.SwerveModuleName;
import frc.robot.util.PIDFFGains;
import lombok.experimental.UtilityClass;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@UtilityClass
public final class Constants {

  public static final boolean TUNING_MODE = false;
  public static final boolean ENABLE_VISION_POSE_ESTIMATION = true;
  public static final int CAN_TIMEOUT_MS = 200;

  @UtilityClass
  public static final class Logging {
    public static final String sda1Dir = "/media/sda1";
    public static final String sda2Dir = "/media/sda2";
  }

  public final class LimeLightConstants {
    public static double CAMERA_TO_TAG_MAX_DIST_INCHES = 120;
    public static double VISION_STD_DEVI_POSITION_IN_METERS = 0.9;
    public static double VISION_STD_DEVI_ROTATION_IN_RADIANS = Units.degreesToRadians(5);
    public static double MAX_POSE_JUMP_IN_INCHES = 6 * 12;
  }

  @UtilityClass
  public static final class RobotMap {
    public static final int PIGEON_CAN_ID = 20;

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  @UtilityClass
  public static final class DriveConstants {

    public static final double K_JOYSTICK_TURN_DEADZONE = 0.04;
    public static final double WHEEL_DIAMETER = 3.9; // 4.02267; // 3.85;
    public static final double GEAR_RATIO = 6.12;
    public static final double DIST_PER_PULSE =
        (1.0 / GEAR_RATIO) * Units.inchesToMeters(WHEEL_DIAMETER) * Math.PI;
    // 1;
    public static final double MAX_SWERVE_VEL = Units.feetToMeters(15.0);
    public static final double MAX_SWERVE_VEL_AUTO = Units.feetToMeters(12.0);
    public static final double MAX_SWERVE_AZI = Math.PI;
    public static final double MAX_SWERVE_ACCEL = Units.feetToMeters(5);
    public static final double MAX_ROTATIONAL_SPEED_RAD_PER_SEC = Units.degreesToRadians(275);

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int AZI_CURRENT_LIMIT = 20;

    public static final double K_MODULE_DISTANCE_FROM_CENTER = Units.inchesToMeters(20.75 / 2);

    public static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    public static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    public static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    public static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);

    private static final double BUMPERLESS_ROBOT_LENGTH = Units.inchesToMeters(26.5);
    private static final double BUMPERLESS_ROBOT_WIDTH = Units.inchesToMeters(26.5);
    private static final double BUMPER_THICKNESS = Units.inchesToMeters(3);

    public static final double FULL_ROBOT_WIDTH = BUMPERLESS_ROBOT_WIDTH + BUMPER_THICKNESS * 2;
    public static final double FULL_ROBOT_LENGTH = BUMPERLESS_ROBOT_LENGTH + BUMPER_THICKNESS * 2;

    public static final double HEADING_CONTROLLER_DRIVER_CHANGE_RATE = 4;
    public static final PIDFFGains K_HEADING_CONTROLLER_GAINS =
        PIDFFGains.builder().name("Heading Controller").kP(12).kD(.35).kS(3).build();

    public static final ModuleInfo FRONT_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(1)
            .aziCANId(2)
            .aziEncoderCANId(0)
            .offset(0.312)
            .location(FRONT_LEFT_LOCATION)
            .build();

    public static final ModuleInfo FRONT_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(3)
            .aziCANId(4)
            .aziEncoderCANId(1)
            .offset(0.527)
            .location(FRONT_RIGHT_LOCATION)
            .build();

    public static final ModuleInfo BACK_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(10)
            .aziCANId(9)
            .aziEncoderCANId(2)
            .offset(0.857)
            .location(BACK_LEFT_LOCATION)
            .build();

    public static final ModuleInfo BACK_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(5)
            .aziCANId(6)
            .aziEncoderCANId(3)
            .offset(0.864)
            .location(BACK_RIGHT_LOCATION)
            .build();

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains K_DEFAULT_AZIMUTH_GAINS =
          PIDFFGains.builder().name("Swerve/Defaults/Azimuth").kP(0.12).build().buildTunables();

      public static final PIDFFGains K_DEFAULT_DRIVING_GAINS =
          PIDFFGains.builder()
              .name("Swerve/Defaults/Driving")
              .kP(1.0)
              .kS(0.225)
              .kV(2.33)
              .build()
              .buildTunables();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_X =
          PIDFFGains.builder().name("Trajectory/X").kP(7).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_Y =
          PIDFFGains.builder().name("Trajectory/Y").kP(7).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_ROTATION =
          PIDFFGains.builder().name("Trajectory/R").kP(2.5).build();
    }
  }
}
