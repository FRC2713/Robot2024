// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.fullRoutines.AmpSide;
import frc.robot.commands.fullRoutines.BottomTwo;
import frc.robot.commands.fullRoutines.FourPieceCentre;
import frc.robot.commands.fullRoutines.FourPieceL;
import frc.robot.commands.fullRoutines.NonAmpSide;
import frc.robot.commands.otf.OTFAmp;
import frc.robot.commands.otf.RotateScore;
import frc.robot.subsystems.candle.NewCandle;
import frc.robot.subsystems.candle.NewCandle.LightCode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
import frc.robot.subsystems.elevatorIO.ElevatorIOSparks;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.intakeIO.IntakeIOSim;
import frc.robot.subsystems.intakeIO.IntakeIOSparks;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.ShooterIOSim;
import frc.robot.subsystems.shooterIO.ShooterIOVortexVortexLS;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.shooterPivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooterPivot.ShooterPivotIOSparks;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOKrakenNeo;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.visionIO.LimelightGP;
import frc.robot.subsystems.visionIO.Vision;
import frc.robot.subsystems.visionIO.VisionIO.LEDMode;
import frc.robot.subsystems.visionIO.VisionIOLimelightLib;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.util.ChangeDetector;
import frc.robot.util.MechanismManager;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.RumbleManager;
import frc.robot.util.SwerveHeadingController;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
  public static LimelightGP visionGP;
  public static Vision visionLeft, visionRight;
  public static SwerveSubsystem swerveDrive;
  public static ShooterPivot shooterPivot;
  public static Elevator elevator;
  public static Shooter shooter;
  public static Intake intake;
  //   public static Candle candle;
  public static NewCandle candle;

  private LinearFilter canUtilizationFilter = LinearFilter.singlePoleIIR(0.25, 0.02);

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  private Command autoCommand;
  private final LoggedDashboardChooser<RHRFullRoutine> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");

  private ChangeDetector<Optional<Alliance>> allianceChangeDetector;
  private ChangeDetector<RHRFullRoutine> autoChangeDetector;
  private Rotation2d gyroInitial = Rotation2d.fromRadians(0);

  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    URCL.start();
    Logger.recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    elevator = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
    shooter = new Shooter(isSimulation() ? new ShooterIOSim() : new ShooterIOVortexVortexLS());
    shooterPivot =
        new ShooterPivot(isSimulation() ? new ShooterPivotIOSim() : new ShooterPivotIOSparks());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());

    candle = new NewCandle(isSimulation());

    candle.setLEDColor(LightCode.RESTING_RED);

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

    visionLeft =
        new Vision(
            isSimulation()
                ? new VisionIOSim(LimeLightConstants.LEFT_LIMELIGHT_INFO)
                : new VisionIOLimelightLib(LimeLightConstants.LEFT_LIMELIGHT_INFO));

    visionRight =
        new Vision(
            isSimulation()
                ? new VisionIOSim(LimeLightConstants.RIGHT_LIMELIGHT_INFO)
                : new VisionIOLimelightLib(LimeLightConstants.RIGHT_LIMELIGHT_INFO));

    visionGP = new LimelightGP(Constants.LimeLightConstants.GP_LIMELIGHT_INFO, isSimulation());

    mechManager = new MechanismManager();

    createDriverBindings();
    createOperatorBindings();
    createAutomaticTriggers();

    allianceChangeDetector =
        new ChangeDetector<>(
            (c) -> {
              seedGyroBasedOnAlliance();
              buildAutoChooser();
              RotateScore.updateSpeakerLoc();
              RotateScore.updateAmpLoc();
            });

    autoChangeDetector =
        new ChangeDetector<>(
            (auto) -> {
              gyroInitial = auto.traj1.getInitialPose().getRotation();
              seedGyroBasedOnAlliance();
            });
  }

  public void createDriverBindings() {
    driver
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    Cmds.setState(ShooterPivot.State.INTAKING),
                    Cmds.setState(Elevator.State.MIN_HEIGHT)),
                new WaitUntilCommand(
                    () -> elevator.atTargetHeight() && shooterPivot.isAtTargetAngle()),
                Commands.parallel(
                    Cmds.setState(Intake.State.INTAKE_GP),
                    Cmds.setState(Shooter.State.INTAKING),
                    Cmds.setState(ShooterPivot.State.INTAKING)),
                Commands.waitUntil(() -> shooter.hasGamePiece()),
                Commands.parallel(
                    new ScheduleCommand(RumbleManager.driverBigOneSec()),
                    Commands.sequence(
                        Cmds.setState(Intake.State.OFF), Cmds.setState(Shooter.State.HOLDING_GP)))))
        .onFalse(
            Commands.parallel(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.OFF),
                    new InstantCommand(),
                    () -> shooter.getState() == Shooter.State.INTAKING)));

    driver
        .a()
        .onTrue(
            Commands.sequence(
                Cmds.setState(Elevator.State.MIN_HEIGHT),
                Cmds.setState(Intake.State.INTAKE_GP),
                Cmds.setState(Shooter.State.INTAKING),
                Cmds.setState(ShooterPivot.State.INTAKING),
                new InstantCommand(
                    () -> {
                      Robot.swerveDrive.setMotionMode(MotionMode.DRIVE_TOWARDS_GP);
                      VehicleState.getInstance().resetClosestGP();
                    })))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.OFF),
                    new InstantCommand(),
                    () -> shooter.getState() == Shooter.State.INTAKING),
                new InstantCommand(
                    () -> {
                      Robot.swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);
                      VehicleState.getInstance().resetClosestGP();
                    })));

    driver
        .leftTrigger(0.3)
        .whileTrue(
            Commands.sequence(
                Cmds.setState(MotionMode.HEADING_CONTROLLER),
                new InstantCommand(
                    () ->
                        SwerveHeadingController.getInstance()
                            .setSetpoint(
                                RotateScore.getOptimalAmpAngle(Robot.swerveDrive.getUsablePose()))),
                Cmds.setState(ShooterPivot.State.FEEDER_SHOT),
                Cmds.setState(Shooter.State.FEEDER_SHOT),
                new WaitUntilCommand(
                    () ->
                        shooter.isAtTarget()
                            && SwerveHeadingController.getInstance().atSetpoint(0.3)),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                // Cmds.setState(MotionMode.FULL_DRIVE),
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece()),
                new WaitCommand(0.05),
                Cmds.setState(ShooterPivot.State.INTAKING)));

    driver
        .rightBumper()
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.FENDER_SHOT),
                Cmds.setState(Shooter.State.FENDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece()),
                new WaitCommand(0.05),
                Cmds.setState(ShooterPivot.State.INTAKING)));

    driver
        .rightTrigger(0.3)
        .whileTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> VehicleState.getInstance().setShouldUpdateCenterTagAlignment(true)),
                Cmds.setState(ShooterPivot.State.POSE_AIM),
                Cmds.setState(Shooter.State.PODIUM_SHOT),
                Cmds.setState(MotionMode.ALIGN_TO_TAG),
                new WaitUntilCommand(
                    () ->
                        shooter.isAtTarget()
                            && SwerveHeadingController.getInstance().atSetpoint(0.3)),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                Cmds.setState(MotionMode.FULL_DRIVE),
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.PODIUM_SHOT),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setModeAndWait(ShooterPivot.State.INTAKING)));

    driver
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
                }));
    driver
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
                }));

    driver
        .povUp()
        .onTrue(
            new SequentialCommandGroup(
                Cmds.setState(SwerveSubsystem.MotionMode.HEADING_CONTROLLER),
                SwerveSubsystem.Commands.setHeading(
                    Rotation2d.fromDegrees(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? 0
                            : 180))));

    driver
        .povLeft()
        .onTrue(
            new SequentialCommandGroup(
                Cmds.setState(SwerveSubsystem.MotionMode.HEADING_CONTROLLER),
                SwerveSubsystem.Commands.setHeading(
                    Rotation2d.fromDegrees(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? 90
                            : 270))));

    driver
        .povRight()
        .onTrue(
            new SequentialCommandGroup(
                Cmds.setState(SwerveSubsystem.MotionMode.HEADING_CONTROLLER),
                SwerveSubsystem.Commands.setHeading(
                    Rotation2d.fromDegrees(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? 270
                            : 90))));

    driver
        .povDown()
        .onTrue(
            new SequentialCommandGroup(
                Cmds.setState(SwerveSubsystem.MotionMode.HEADING_CONTROLLER),
                SwerveSubsystem.Commands.setHeading(
                    Rotation2d.fromDegrees(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? 180
                            : 0))));

    driver
        .x()
        .whileTrue(
            Commands.sequence(
                new InstantCommand(() -> Robot.swerveDrive.setMotionMode(MotionMode.TRAJECTORY)),
                Commands.parallel(
                    OTFAmp.getInstance().run(),
                    Cmds.setState(Elevator.State.AMP),
                    Cmds.setState(ShooterPivot.State.AMP_SHOT),
                    new WaitUntilCommand(elevator::atTargetHeight),
                    new WaitUntilCommand(shooterPivot::isAtTargetAngle))))
        .onFalse(new InstantCommand(() -> Robot.swerveDrive.setMotionMode(MotionMode.FULL_DRIVE)));

    driver
        .y()
        .onTrue(Cmds.setState(Shooter.State.AMP_SHOT))
        .onFalse(
            Commands.either(
                Cmds.setState(Shooter.State.HOLDING_GP),
                Cmds.setState(Shooter.State.OFF),
                () -> shooter.getState() == Shooter.State.FENDER_SHOT));
  }

  public void createOperatorBindings() {
    operator
        .x()
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.FENDER_SHOT),
                Cmds.setState(Shooter.State.FENDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.FENDER_SHOT),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setModeAndWait(ShooterPivot.State.INTAKING)));

    operator
        .a()
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.PODIUM_SHOT),
                Cmds.setState(Shooter.State.PODIUM_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.PODIUM_SHOT),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setModeAndWait(ShooterPivot.State.INTAKING)));

    operator
        .povUp()
        .onTrue(
            Commands.sequence(
                Cmds.setState(Elevator.State.MAX_HEIGHT),
                Cmds.setState(ShooterPivot.State.PREP_FOR_CLIMB)));
    operator.povDown().onTrue(Commands.sequence(Cmds.setState(Elevator.State.MIN_HEIGHT)));

    operator
        .povLeft()
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.FEEDER_SHOT),
                Cmds.setState(Shooter.State.FEEDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Cmds.setState(Intake.State.INTAKE_GP),
                RedHawkUtil.logShot()))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece()),
                new WaitCommand(0.05),
                Cmds.setState(ShooterPivot.State.INTAKING)));

    operator
        .rightBumper()
        .onTrue(
            Commands.sequence(
                Cmds.setState(Elevator.State.AMP),
                Cmds.setState(ShooterPivot.State.AMP_SHOT),
                new WaitUntilCommand(elevator::atTargetHeight),
                new WaitUntilCommand(shooterPivot::isAtTargetAngle),
                Cmds.setState(Shooter.State.AMP_SHOT)))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Intake.State.OFF),
                Cmds.setState(Elevator.State.MIN_HEIGHT),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece()),
                Cmds.setState(ShooterPivot.State.INTAKING)));

    operator
        .leftTrigger(0.3)
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.INTAKING),
                Cmds.setState(Elevator.State.MIN_HEIGHT),
                Cmds.setState(Shooter.State.OUTTAKE_FORWARD),
                Cmds.setState(Intake.State.INTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece())));

    operator
        .rightTrigger(0.3)
        .onTrue(
            Commands.sequence(
                Cmds.setState(ShooterPivot.State.INTAKING),
                Cmds.setState(Elevator.State.MIN_HEIGHT),
                new WaitUntilCommand(elevator::atTargetHeight),
                new WaitUntilCommand(shooterPivot::isAtTargetAngle),
                Cmds.setState(Shooter.State.OUTTAKE_BACKWARDS),
                Cmds.setState(Intake.State.OUTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece())));

    operator
        .b()
        .onTrue(Commands.sequence(Cmds.setState(Shooter.State.FORCE_MANUAL_CONTROL)))
        .onFalse(
            Commands.sequence(
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece())));

    operator
        .y()
        .onTrue(
            Commands.sequence(
                Cmds.setState(Elevator.State.ELEVATORSHOT),
                Cmds.setState(ShooterPivot.State.POSE_AIM_ELEVATOR_SHOT),
                Cmds.setState(Shooter.State.ELEVATOR_SHOT)))
        .onFalse(
            Commands.sequence(
                Cmds.setState(Elevator.State.ELEVATORSHOT),
                Commands.either(
                    Cmds.setState(Shooter.State.HOLDING_GP),
                    Cmds.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece())));

    // operator
    //     .povLeft()
    //     .onTrue(
    //         Commands.sequence(
    //             Cmds.setState(Intake.State.FULL_OUT),
    //             Cmds.setState(Shooter.State.FULL_OUT)))
    //     .onFalse(
    //         Commands.sequence(
    //             Commands.either(
    //                 Cmds.setState(Shooter.State.HOLDING_GP),
    //                 Cmds.setState(Shooter.State.OFF),
    //                 () -> shooter.hasGamePiece())));

    // operator
    //     .povRight()
    //     .onTrue(
    //         Commands.sequence(
    //             Cmds.setState(Intake.State.FULL_IN),
    //             Cmds.setState(Shooter.State.FULL_IN)))
    //     .onFalse(
    //         Commands.sequence(
    //             Commands.either(
    //                 Cmds.setState(Shooter.State.HOLDING_GP),
    //                 Cmds.setState(Shooter.State.OFF),
    //                 () -> shooter.hasGamePiece())));

    operator
        .start()
        .onTrue(Cmds.setState(Intake.State.NOTE_IN_CHASSIS))
        .onFalse(Cmds.setState(Intake.State.OFF));

    operator
        .back()
        .onTrue(Cmds.setState(Intake.State.OUTAKE_GP))
        .onFalse(Cmds.setState(Intake.State.OFF));

    // operator
    //     .back()
    //     .onTrue(
    //         Commands.sequence(
    //             Cmds.setState(Intake.State.CLEANING),
    //             Cmds.setState(ShooterPivot.State.CLEANING),
    //             Cmds.setState(Shooter.State.CLEANING)));
  }

  public void createAutomaticTriggers() {
    new Trigger(() -> shooter.hasGamePiece())
        .onTrue(
            Commands.sequence(
                NewCandle.Commands.setLEDColor(LightCode.HAS_NOTE),
                new InstantCommand(
                    () -> {
                      visionRight.setLEDMode(LEDMode.FORCE_BLINK);
                      visionLeft.setLEDMode(LEDMode.FORCE_BLINK);
                    }),
                new WaitCommand(2),
                new InstantCommand(
                    () -> {
                      visionRight.setLEDMode(LEDMode.PIPELINE);
                      visionLeft.setLEDMode(LEDMode.PIPELINE);
                    })))
        .onFalse(NewCandle.Commands.setLEDColor(LightCode.OFF));

    new Trigger(
            () ->
                visionGP.detections.length > 0
                    && !shooter.hasGamePiece()
                    && !VehicleState.getInstance().hasGPLock)
        .onTrue(NewCandle.Commands.setLEDColor(LightCode.SEES_NOTE))
        .onFalse(NewCandle.Commands.setLEDColor(LightCode.OFF));

    new Trigger(() -> !shooter.hasGamePiece() && VehicleState.getInstance().hasGPLock)
        .onTrue(NewCandle.Commands.setLEDColor(LightCode.LOCKED_ON_NOTE));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // RumbleManager.getInstance().periodic();
    mechManager.periodic();
    updatePreMatchDashboardValues();

    if (Math.abs(driver.getRightX()) > 0.25
        && swerveDrive.getMotionMode() != MotionMode.DRIVE_TOWARDS_GP) {
      swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);
    }

    RobotController.setBrownoutVoltage(6.4);
    if (RobotController.isBrownedOut()) {
      //   swerveDrive.setDriveCurrentLimits(20);
    }

    // swerveDrive.seed();

    RotateScore.getOptimalShooterAngle(Robot.swerveDrive.getUsablePose());

    Logger.recordOutput(
        "Filtered CAN Utilization",
        canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    Logger.recordOutput(
        "Memory Usage",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);

    VehicleState.getInstance()
        .updateDynamicPivotAngle(visionLeft.getInputs(), visionRight.getInputs());
    RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());

    swerveDrive.updatePoseEstimatorWithVisionBotPose(visionLeft.getInfo(), visionLeft.getInputs());
    swerveDrive.updatePoseEstimatorWithVisionBotPose(
        visionRight.getInfo(), visionRight.getInputs());
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    swerveDrive.seed();
    swerveDrive.setMotionMode(MotionMode.LOCKDOWN);
  }

  @Override
  public void disabledPeriodic() {
    swerveDrive.seed();
    allianceChangeDetector.feed(DriverStation.getAlliance());
    autoChangeDetector.feed(autoChooser.get());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void buildAutoChooser() {
    autoChooser.addDefaultOption("NonAmpSide", new NonAmpSide());
    autoChooser.addOption("BottomTwo", new BottomTwo());
    autoChooser.addOption("FourPieceCentre", new FourPieceCentre());
    autoChooser.addOption("FourPieceL", new FourPieceL());
    autoChooser.addOption("AmpSide", new AmpSide());
  }

  public void updatePreMatchDashboardValues() {
    var encoderReadings = swerveDrive.getAbsoluteEncoderAngles();
    SmartDashboard.putNumber("Dashboard/Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean(
        "Dashboard/Has Alliance Color", DriverStation.getAlliance().isPresent());
    SmartDashboard.putBoolean("Dashboard/Front Left Encoder Good", encoderReadings[0] != 0.0);
    SmartDashboard.putBoolean("Dashboard/Front Right Encoder Good", encoderReadings[1] != 0.0);
    SmartDashboard.putBoolean("Dashboard/Back Left Encoder Good", encoderReadings[2] != 0.0);
    SmartDashboard.putBoolean("Dashboard/Back Right Encoder Good", encoderReadings[3] != 0.0);
    SmartDashboard.putNumber("Dashboard/Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Dashboard/Gyro Yaw", swerveDrive.getYaw().getDegrees());
    SmartDashboard.putString("Dashboard/States/Elevator", elevator.getState().name());
    SmartDashboard.putString("Dashboard/States/Intake", intake.getState().name());
    SmartDashboard.putString("Dashboard/States/Shooter", shooter.getState().name());
    SmartDashboard.putString("Dashboard/States/Pivot", shooterPivot.getState().name());
    SmartDashboard.putString("Dashboard/States/Swerve", swerveDrive.getMotionMode().name());
    SmartDashboard.putNumber("Dashboard/Elevator Left", elevator.getLeftPosition());
    SmartDashboard.putNumber("Dashboard/Elevator Right", elevator.getRightPosition());
    SmartDashboard.putNumber("Dashboard/Pivot Left", shooterPivot.getLeftPosition());
    SmartDashboard.putNumber("Dashboard/Pivot Right", shooterPivot.getRightPosition());
  }

  public void seedGyroBasedOnAlliance() {
    Optional<Alliance> checkedAlliance = DriverStation.getAlliance();
    var startingAngle = gyroInitial;

    // if we are on blue, we are probably facing towards the blue DS, which is -x.
    // that corresponds to a 180 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Blue) {
      swerveDrive.resetGyro(startingAngle);
      SwerveSubsystem.allianceFlipper = 1;
    }

    // if we are on red, we are probably facing towards the red DS, which is +x.
    // that corresponds to a 0 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Red) {
      swerveDrive.resetGyro(RedHawkUtil.Reflections.reflect(startingAngle));
      SwerveSubsystem.allianceFlipper = -1;
    }
  }

  @Override
  public void driverStationConnected() {

    seedGyroBasedOnAlliance();
    buildAutoChooser();
    RedHawkUtil.logShotFirst();
    // visionFront.setPriorityId(
    // switch (DriverStation.getAlliance().get()) {
    // case Blue -> 7;
    // case Red -> 3;
    // });
  }
}
