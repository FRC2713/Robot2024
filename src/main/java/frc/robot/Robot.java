// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.fullRoutines.RHRNamedCommands;
import frc.robot.commands.fullRoutines.SelfishAuto;
import frc.robot.commands.fullRoutines.Simple;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.subsystems.visionIO.Vision;
import frc.robot.subsystems.visionIO.VisionIOLimelight;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.subsystems.visionIO.VisionManager;
import frc.robot.util.MechanismManager;
import frc.robot.util.RedHawkUtil;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
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

  DoubleArraySubscriber frontCamera2TagPose;
  DoubleArraySubscriber rearCamera2TagPose;

  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    // URCL.start();
    Logger.recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(RedHawkUtil.getLogDirectory()));
    }

    Logger.start();

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

    VisionManager visionManager =
        new VisionManager(
            new Vision(
                "Front",
                isSimulation()
                    ? new VisionIOSim("limelight-front")
                    : new VisionIOLimelight("limelight-front")),
            new Vision(
                "Rear",
                isSimulation()
                    ? new VisionIOSim("limelight-rear")
                    : new VisionIOLimelight("limelight-rear")));

    mechManager = new MechanismManager();

    checkAlliance();
    buildAutoChooser();

    // driver
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
    //               SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
    //             }));

    // driver
    //     .povLeft()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
    //               SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
    //             }));

    // driver
    //     .povDown()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
    //               SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
    //             }));

    // driver
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
    //               SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
    //             }));

    // driver
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.setMotionMode(MotionMode.LOCKDOWN);
    //             }));

    // driver
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
    //             }));

    // driver
    //     .back()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
    //             }));

    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // ErrHandler.getInstance().log();
    // RumbleManager.getInstance().periodic();
    // mechManager.periodic();
    if (Math.abs(driver.getRightX()) > 0.25) {
      swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);
    }

    // swerveDrive.seed();

    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(swerveDrive.getTotalCurrentDraw()));

    Logger.recordOutput(
        "Filtered CAN Utilization",
        canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    Logger.recordOutput(
        "Memory Usage",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);

    // TimestampedDoubleArray[] frontfQueue = frontVisionPose.readQueue();
    // TimestampedDoubleArray[] frontcQueue = frontCamera2TagPose.readQueue();

    // TimestampedDoubleArray[] rearfQueue = rearVisionPose.readQueue();
    // TimestampedDoubleArray[] rearcQueue = rearCamera2TagPose.readQueue();

    // if (frontfQueue.length > 0
    //     && frontcQueue.length > 0
    //     && vision.hasMultipleTargets(Limelights.FRONT)) {
    //   TimestampedDoubleArray fLastCameraReading = frontfQueue[frontfQueue.length - 1];
    //   TimestampedDoubleArray cLastCameraReading = frontcQueue[frontcQueue.length - 1];
    //   swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    // } else if (rearfQueue.length > 0
    //     && rearcQueue.length > 0
    //     && vision.hasMultipleTargets(Limelights.REAR)) {
    //   TimestampedDoubleArray fLastCameraReading = rearfQueue[rearfQueue.length - 1];
    //   TimestampedDoubleArray cLastCameraReading = rearcQueue[rearcQueue.length - 1];
    //   swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    // }
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    swerveDrive.seed();
    swerveDrive.setMotionMode(MotionMode.LOCKDOWN);
    // vision.setCurrentSnapshotMode(SnapshotMode.OFF);
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

    // vision.setCurrentSnapshotMode(SnapshotMode.TWO_PER_SECOND);
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
    RHRNamedCommands.registerGenericCommands();

    // SwerveSubsystem.allianceFlipper = DriverStation.getAlliance() == Alliance.Red ? -1 : 1;
    autoChooser.addDefaultOption("Simple", Simple.getAutonomousCommand());
    autoChooser.addOption("Selfish", SelfishAuto.getAutonomousCommand());
  }

  public void checkAlliance() {
    Optional<Alliance> checkedAlliance = DriverStation.getAlliance();
    Logger.recordOutput("DS Alliance has value", checkedAlliance.isPresent());
    if (checkedAlliance.isPresent()) {
      Logger.recordOutput("DS Alliance value", checkedAlliance.get());
      // buildAutoChooser();
    }

    // these gyro resets are mostly for ironing out teleop driving issues

    // if we are on blue, we are probably facing towards the blue DS, which is -x.
    // that corresponds to a 180 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Blue) {
      swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
    }

    // if we are on red, we are probably facing towards the red DS, which is +x.
    // that corresponds to a 0 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Red) {
      swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
    }
  }
}
