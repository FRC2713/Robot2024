// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.fullRoutines.Simple;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.subsystems.visionIO.Vision;
import frc.robot.subsystems.visionIO.Vision.Limelights;
import frc.robot.subsystems.visionIO.Vision.SnapshotMode;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.subsystems.visionIO.VisionLimelight;
import frc.robot.util.MechanismManager;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.RedHawkUtil.ErrHandler;
import frc.robot.util.RumbleManager;
import frc.robot.util.SwerveHeadingController;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
  public static Vision vision;
  //   public static Slapper slapper;
  public static SwerveSubsystem swerveDrive;
  private Command autoCommand;
  private LinearFilter canUtilizationFilter = LinearFilter.singlePoleIIR(0.25, 0.02);

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");

  public static double[] poseValue;
  DoubleArraySubscriber frontVisionPose;
  DoubleArraySubscriber rearVisionPose;

  Alliance currentAlliance = Alliance.Invalid;
  DoubleArraySubscriber frontCamera2TagPose;
  DoubleArraySubscriber rearCamera2TagPose;

  @Override
  public void robotInit() {
    NetworkTable frontTable =
        NetworkTableInstance.getDefault().getTable(Vision.Limelights.FRONT.table);
    NetworkTable rearTable =
        NetworkTableInstance.getDefault().getTable(Vision.Limelights.REAR.table);
    frontVisionPose = frontTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    frontCamera2TagPose =
        frontTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    rearVisionPose = rearTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    rearCamera2TagPose =
        rearTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.getInstance().recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.getInstance().recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      File sda1 = new File(Constants.Logging.sda1Dir);
      File sda2 = new File(Constants.Logging.sda2Dir);

      if (sda1.exists() && sda1.isDirectory()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter(Constants.Logging.sda1Dir));
        Logger.getInstance().recordOutput("isLoggingToUsb", true);
      } else {
        RedHawkUtil.ErrHandler.getInstance()
            .addError(
                "Cannot log to "
                    + Constants.Logging.sda1Dir
                    + ", trying "
                    + Constants.Logging.sda2Dir);
        if (sda2.exists() && sda2.isDirectory()) {
          Logger.getInstance().addDataReceiver(new WPILOGWriter(Constants.Logging.sda2Dir));
          Logger.getInstance().recordOutput("isLoggingToUsb", true);
        } else {
          RedHawkUtil.ErrHandler.getInstance()
              .addError("Cannot log to " + Constants.Logging.sda2Dir);
          Logger.getInstance().recordOutput("isLoggingToUsb", false);
        }
      }
    } else {
      Logger.getInstance().recordOutput("isLoggingToUsb", false);
    }

    Logger.getInstance().start();

    vision =
        new Vision(
            isSimulation() ? new VisionIOSim() : new VisionLimelight("limelight"),
            isSimulation() ? new VisionIOSim() : new VisionLimelight("limelight-rear"));
    // slapper = new Slapper(true ? new SlapperIOSim() : new SlapperIOSparks());

    // fourBar = new FourBar(true ? new FourBarIOSim() : new FourBarIOSparks());
    // elevator = new Elevator(true ? new ElevatorIOSim() : new ElevatorIOSparks());
    // intake = new Intake(true ? new IntakeIOSim() : new IntakeIOSparks());
    // vision = new Vision(true ? new VisionIOSim() : new VisionLimelight());

    swerveDrive =
        isSimulation()
            // true
            ? new SwerveSubsystem(
                new SwerveIOSim(),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_RIGHT))
            : new SwerveSubsystem(
                new SwerveIOPigeon2(),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.BACK_RIGHT));

    mechManager = new MechanismManager();

    checkAlliance();
    buildAutoChooser();

    driver
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
                }));

    driver
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
                }));

    driver
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
                }));

    driver
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
                }));

    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.setMotionMode(MotionMode.LOCKDOWN);
                }));

    driver
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
                }));

    driver
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
                }));

    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ErrHandler.getInstance().log();
    RumbleManager.getInstance().periodic();
    mechManager.periodic();
    if (Math.abs(driver.getRightX()) > 0.25) {
      swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);
    }

    // swerveDrive.seed();

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(swerveDrive.getTotalCurrentDraw()));

    Logger.getInstance()
        .recordOutput(
            "Filtered CAN Utilization",
            canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    Logger.getInstance()
        .recordOutput(
            "Memory Usage",
            (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory())
                / 1024.0
                / 1024.0);

    TimestampedDoubleArray[] frontfQueue = frontVisionPose.readQueue();
    TimestampedDoubleArray[] frontcQueue = frontCamera2TagPose.readQueue();

    TimestampedDoubleArray[] rearfQueue = rearVisionPose.readQueue();
    TimestampedDoubleArray[] rearcQueue = rearCamera2TagPose.readQueue();

    if (frontfQueue.length > 0
        && frontcQueue.length > 0
        && vision.hasMultipleTargets(Limelights.FRONT)) {
      TimestampedDoubleArray fLastCameraReading = frontfQueue[frontfQueue.length - 1];
      TimestampedDoubleArray cLastCameraReading = frontcQueue[frontcQueue.length - 1];
      swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    } else if (rearfQueue.length > 0
        && rearcQueue.length > 0
        && vision.hasMultipleTargets(Limelights.REAR)) {
      TimestampedDoubleArray fLastCameraReading = rearfQueue[rearfQueue.length - 1];
      TimestampedDoubleArray cLastCameraReading = rearcQueue[rearcQueue.length - 1];
      swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    }
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    swerveDrive.seed();
    swerveDrive.setMotionMode(MotionMode.LOCKDOWN);
    vision.setCurrentSnapshotMode(SnapshotMode.OFF);
  }

  @Override
  public void disabledPeriodic() {
    checkAlliance();

    swerveDrive.seed();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    checkAlliance();
    swerveDrive.setMotionMode(MotionMode.TRAJECTORY);
    autoCommand = autoChooser.get();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);

    vision.setCurrentSnapshotMode(SnapshotMode.TWO_PER_SECOND);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    swerveDrive.zeroGyro();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void buildAutoChooser() {
    SwerveSubsystem.allianceFlipper = DriverStation.getAlliance() == Alliance.Red ? -1 : 1;
    autoChooser.addDefaultOption("Simple", new Simple());
  }

  public void checkAlliance() {
    Alliance checkedAlliance = DriverStation.getAlliance();
    Logger.getInstance().recordOutput("DS Alliance", currentAlliance.name());

    if (DriverStation.isDSAttached() && checkedAlliance != currentAlliance) {
      currentAlliance = checkedAlliance;

      // these gyro resets are mostly for ironing out teleop driving issues

      // if we are on blue, we are probably facing towards the blue DS, which is -x.
      // that corresponds to a 180 deg heading.
      if (checkedAlliance == Alliance.Blue) {
        swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
      }

      // if we are on red, we are probably facing towards the red DS, which is +x.
      // that corresponds to a 0 deg heading.
      if (checkedAlliance == Alliance.Red) {
        swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
      }

      buildAutoChooser();
    }
  }
}
