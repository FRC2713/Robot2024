// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerveIO.module.ModuleInfo;
import frc.robot.subsystems.swerveIO.module.SwerveModuleName;
import frc.robot.subsystems.visionIO.VisionInfo;
import frc.robot.subsystems.visionIO.VisionInfo.MountingDirection;
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

    public static VisionInfo FRONT_LIMELIGHT_INFO =
        VisionInfo.builder()
            .ntTableName("limelight")
            .location(new Transform3d(0.354453, 9.148643, -19.964190, new Rotation3d(0, 75, 90)))
            .mountingDirection(MountingDirection.HORIZONTAL_LL3)
            .build();
    public static VisionInfo REAR_LIMELIGHT_INFO =
        VisionInfo.builder()
            .ntTableName("limelight-rear")
            .location(new Transform3d())
            .mountingDirection(MountingDirection.VERTICAL_LL3)
            .build();
  }

  @UtilityClass
  public static final class RobotMap {
    public static final int PIGEON_CAN_ID = 60;
    public static final int LEFT_ELEVATOR_CAN_ID = 3;
    public static final int RIGHT_ELEVATOR_CAN_ID = 4;

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final int INTAKE_TOF_SENSOR_ID = 70;
    public static final int INTAKE_LEFT_MOTOR_CAN_ID = 5;
    public static final int INTAKE_RIGHT_MOTOR_CAN_ID = 6;
    
    public static final int FRONT_LEFT_AZIMUTH_CAN_ID = 1;
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 41;

    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 42;
    public static final int FRONT_RIGHT_AZIMUTH_CAN_ID = 2;


    public static final int BACK_LEFT_DRIVE_CAN_ID = 43;
    public static final int BACK_LEFT_AZIMUTH_CAN_ID = 3;

    public static final int BACK_RIGHT_DRIVE_CAN_ID = 44;    
    public static final int BACK_RIGHT_AZIMUTH_CAN_ID = 4; 

  }

  public static final class IntakeConstants
  {
    public static final double MOI = 0.0005;
    public static final double LEFT_GEARING = 5.0;
    public static final double RIGHT_GEARING = 5.0;

    public static final double SENSOR_THRESHOLD = 10;
    public static final double MAX_RPM = 5000;
  }

  public static final class ShooterPivotConstants {
    public static final double LENGTH_METERS = Units.inchesToMeters(18);
    public static final double MASS_KG = 6.80389;
    public static final double MAX_ANGLE_DEGREES = 90;
    public static final double RETRACTED_ANGLE_DEGREES = 0;
    public static final boolean SIMULATE_GRAVITY = true;
    public static final double GEARING = 100;
    public static final double STARTING_ANGLE_RADS = Units.degreesToRadians(45);
    public static final int SHOOTER_PIVOT_MAX_CURRENT = 30;
    public static final double MAX_DEGREES_PER_SECOND = 5;
    public static final PIDFFGains SHOOTER_PIVOT_GAINS =
        PIDFFGains.builder().name("ShooterPivot Controller").kP(20).kD(0).kG(0.85).build();
  }

  public static final class ElevatorConstants {
    public static final PIDFFGains ELEVATOR_GAINS =
        PIDFFGains.builder().name("Elevator Controller").kP(0.75).kD(0.0).kG(0.85).build();
    public static final double GEARING = 5.0;
    public static final double CARRIAGE_MASS_KG = 0.3;
    public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1);
    public static final double MIN_HEIGHT_METERS = 0;
    public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(50);
    public static final double STARTING_HEIGHT_METERS = Units.inchesToMeters(2);
    public static final boolean SIMULATE_GRAVITY = true;
    public static final int ELEVATOR_CURRENT_LIMIT = 30;
  }

  public static final class SuperStructure {}

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
        PIDFFGains.builder()
            .name("Heading Controller")
            .kP(12)
            .kD(.35)
            .kS(3)
            .build()
            .buildTunables();

    public static final ModuleInfo FRONT_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(RobotMap.FRONT_LEFT_DRIVE_CAN_ID)
            .aziCANId(RobotMap.FRONT_LEFT_AZIMUTH_CAN_ID)
            .aziEncoderCANId(0)
            .offset(0.4711362627767633)
            .location(FRONT_LEFT_LOCATION)
            .build();

    public static final ModuleInfo FRONT_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(RobotMap.FRONT_RIGHT_DRIVE_CAN_ID)
            .aziCANId(RobotMap.FRONT_RIGHT_AZIMUTH_CAN_ID)
            .aziEncoderCANId(1)
            .offset(0.280788243336548)
            .location(FRONT_RIGHT_LOCATION)
            .build();

    public static final ModuleInfo BACK_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(RobotMap.BACK_LEFT_DRIVE_CAN_ID)
            .aziCANId(RobotMap.BACK_LEFT_AZIMUTH_CAN_ID)
            .aziEncoderCANId(2)
            .offset(0.7232726445483575)
            .location(BACK_LEFT_LOCATION)
            .build();

    public static final ModuleInfo BACK_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(RobotMap.BACK_RIGHT_DRIVE_CAN_ID)
            .aziCANId(RobotMap.BACK_RIGHT_AZIMUTH_CAN_ID)
            .aziEncoderCANId(3)
            .offset(0.8124671353331704)
            .location(BACK_RIGHT_LOCATION)
            .build();

    @UtilityClass
    public static final class FieldTunables {
      public static final int TIME_BETWEEN_REGERATION_SECONDS = 2;
    }

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains K_DEFAULT_AZIMUTH_GAINS =
          PIDFFGains.builder()
              .name("Swerve/Defaults/Azimuth")
              // 0.12
              .kP(0.012)
              .build();
      // .buildTunables();

      public static final PIDFFGains K_DEFAULT_DRIVING_GAINS =
          PIDFFGains.builder()
              .name("Swerve/Defaults/Driving")
              // 0.25
              .kP(0.0)
              // 0.225
              .kS(0)
              // 0.2
              .kV(0.114)
              .build();
      // .buildTunables();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_X =
          PIDFFGains.builder().name("Trajectory/X").kP(3).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_Y =
          PIDFFGains.builder().name("Trajectory/Y").kP(7).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_ROTATION =
          PIDFFGains.builder().name("Trajectory/R").kP(0).build();
    }
  }
}
