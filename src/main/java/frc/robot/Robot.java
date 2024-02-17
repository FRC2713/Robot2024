// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.fullRoutines.RHRNamedCommands;
import frc.robot.commands.fullRoutines.SelfishAuto;
import frc.robot.commands.fullRoutines.SimpleChoreo;
import frc.robot.commands.fullRoutines.SimpleWeekZeroAuto;
import frc.robot.commands.fullRoutines.ThreePiece;
import frc.robot.commands.fullRoutines.ThreePieceChoreo;
import frc.robot.commands.otf.OTF;
import frc.robot.commands.otf.RotateScore;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.feederIO.Feeder;
import frc.robot.subsystems.feederIO.FeederIOSim;
import frc.robot.subsystems.feederIO.FeederIOSparks;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.intakeIO.IntakeIOSim;
import frc.robot.subsystems.intakeIO.IntakeIOSparks;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.ShooterIOSim;
import frc.robot.subsystems.shooterIO.ShooterIOVortex;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.shooterPivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterPivot.ShooterPivotIOSparks;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOKrakenNeo;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.visionIO.Vision;
import frc.robot.subsystems.visionIO.VisionIOLimelight;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.util.MechanismManager;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
  private OTF otf = new OTF();

  public static Vision visionFront, visionRear;
  public static SwerveSubsystem swerveDrive;
  public static ShooterPivot shooterPivot;
  public static Elevator elevator;
  public static Shooter shooter;
  public static Intake intake;
  public static Feeder feeder;

  private LinearFilter canUtilizationFilter = LinearFilter.singlePoleIIR(0.25, 0.02);

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  private Command autoCommand;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");

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
      // Logger.addDataReceiver(new WPILOGWriter(RedHawkUtil.getLogDirectory()));
    }

    Logger.start();

    // elevator = new Elevator(true ? new ElevatorIOSim() : null);
    shooter = new Shooter(isSimulation() ? new ShooterIOSim() : new ShooterIOVortex());
    shooterPivot =
        new ShooterPivot(isSimulation() ? new ShooterPivotIOSim() : new ShooterPivotIOSparks());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());
    feeder = new Feeder(isSimulation() ? new FeederIOSim() : new FeederIOSparks());

    swerveDrive =
        isSimulation()
            ? new SwerveSubsystem(
                new SwerveIOSim(),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_RIGHT))
            : new SwerveSubsystem(
                new SwerveIOPigeon2(),
                new SwerveModuleIOKrakenNeo(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOKrakenNeo(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOKrakenNeo(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOKrakenNeo(Constants.DriveConstants.BACK_RIGHT));

    visionFront =
        new Vision(
            isSimulation()
                ? new VisionIOSim(LimeLightConstants.FRONT_LIMELIGHT_INFO)
                : new VisionIOLimelight(LimeLightConstants.FRONT_LIMELIGHT_INFO));
    visionRear =
        new Vision(
            isSimulation()
                ? new VisionIOSim(LimeLightConstants.REAR_LIMELIGHT_INFO)
                : new VisionIOLimelight(LimeLightConstants.REAR_LIMELIGHT_INFO));

    // mechManager = new MechanismManager();

    checkAlliance();

    buildAutoChooser();

    driver
        .rightBumper()
        .onTrue(Intake.Commands.setMotionMode(Intake.MotionMode.INTAKE_GP))
        .onFalse(Intake.Commands.setMotionMode(Intake.MotionMode.OFF));

    driver
        .rightTrigger(0.5)
        .whileTrue(
            Commands.sequence(
                Shooter.Commands.setMotionMode(Shooter.MotionMode.FENDER_SHOT_CLOSED_LOOP),
                new WaitUntilCommand(shooter::isAtTarget),
                Feeder.Commands.setMotionMode(Feeder.MotionMode.SEND_TO_SHOOTER),
                Intake.Commands.setMotionMode(Intake.MotionMode.SEND_GP_TO_FEEDER)))
        .whileFalse(
            Commands.sequence(
                Feeder.Commands.setMotionMode(Feeder.MotionMode.OFF),
                Shooter.Commands.setMotionMode(Shooter.MotionMode.OFF),
                Intake.Commands.setMotionMode(Intake.MotionMode.OFF)));

    driver
        .y()
        .whileTrue(
            Commands.sequence(
                Shooter.Commands.setMotionMode(Shooter.MotionMode.FENDER_SHOT_CLOSED_LOOP),
                new WaitUntilCommand(shooter::isAtTarget),
                Feeder.Commands.setMotionMode(Feeder.MotionMode.SEND_TO_SHOOTER),
                Intake.Commands.setMotionMode(Intake.MotionMode.SEND_GP_TO_FEEDER)))
        .whileFalse(
            Commands.sequence(
                Feeder.Commands.setMotionMode(Feeder.MotionMode.OFF),
                Shooter.Commands.setMotionMode(Shooter.MotionMode.OFF),
                Intake.Commands.setMotionMode(Intake.MotionMode.OFF)));

    driver
        .leftBumper()
        .onTrue(Commands.sequence(Intake.Commands.setMotionMode(Intake.MotionMode.OUTAKE_GP)))
        .onFalse(Intake.Commands.setMotionMode(Intake.MotionMode.OFF));
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

    // RoboRioSim.setVInVoltage(
    // BatterySim.calculateDefaultBatteryLoadedVoltage(swerveDrive.getTotalCurrentDraw()));

    Logger.recordOutput(
        "Filtered CAN Utilization",
        canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    Logger.recordOutput(
        "Memory Usage",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);

    // swerveDrive.updateOdometryFromVision(visionFront.getInfo(), visionFront.getInputs());
    // swerveDrive.updateOdometryFromVision(visionRear.getInfo(), visionRear.getInputs());
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

    // swerveDrive.seed();
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
  public void teleopPeriodic() {
    RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());
  }

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

    autoChooser.addOption("ThreePiece", ThreePiece.getAutonomousCommand());
    autoChooser.addOption("SimpleChoreo", SimpleChoreo.getAutonomousCommand());
    autoChooser.addOption("ThreePieceChoreo", new ThreePieceChoreo());
    autoChooser.addOption("Selfish", SelfishAuto.getAutonomousCommand());
    autoChooser.addDefaultOption("SimpleWeekZero", new SimpleWeekZeroAuto());
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
