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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.commands.fullRoutines.NonAmpSide;
import frc.robot.commands.fullRoutines.RHRNamedCommands;
import frc.robot.commands.fullRoutines.SelfishAuto;
import frc.robot.commands.fullRoutines.SimpleChoreo;
import frc.robot.commands.fullRoutines.ThreePiece;
import frc.robot.commands.fullRoutines.ThreePieceChoreo;
import frc.robot.commands.fullRoutines.Week0MobilityChoreo;
import frc.robot.commands.otf.OTF;
import frc.robot.commands.otf.RotateScore;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
import frc.robot.subsystems.elevatorIO.ElevatorIOSparks;
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
import frc.robot.subsystems.visionIO.VisionIO.LEDMode;
import frc.robot.subsystems.visionIO.VisionIOLimelight;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.util.MechanismManager;
import frc.robot.util.SwerveHeadingController;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
  private OTF otf = new OTF();
  public static Vision visionFront, visionRear;
  public static SwerveSubsystem swerveDrive;
  public static ShooterPivot shooterPivot;
  public static Elevator elevator;
  public static Shooter shooter;
  public static Intake intake;

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
    URCL.start();
    Logger.recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      // Logger.addDataReceiver(new WPILOGWriter(RedHawkUtil.getLogDirectory()));
    }

    Logger.start();

    elevator = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
    shooter = new Shooter(isSimulation() ? new ShooterIOSim() : new ShooterIOVortex());
    shooterPivot =
        new ShooterPivot(isSimulation() ? new ShooterPivotIOSim() : new ShooterPivotIOSparks());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());

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

    // visionRear =
    // new Vision(
    // isSimulation()
    // ? new VisionIOSim(LimeLightConstants.REAR_LIMELIGHT_INFO)
    // : new VisionIOLimelight(LimeLightConstants.REAR_LIMELIGHT_INFO));

    mechManager = new MechanismManager();

    createDriverBindings();
    createOperatorBindings();
    createAutomaticTriggers();
  }

  public void createDriverBindings() {
    driver
        .leftBumper()
        .onTrue(
            Commands.sequence(
                    Intake.Commands.setMotionMode(Intake.State.INTAKE_GP),
                    Shooter.Commands.setState(Shooter.State.INTAKING),
                    ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING))
                .repeatedly()
                .until(() -> shooter.hasGamePiece() || intake.state == Intake.State.OFF)
                .andThen(
                    Commands.sequence(
                        Intake.Commands.setMotionMode(Intake.State.OFF),
                        Commands.either(
                            Shooter.Commands.setState(Shooter.State.OFF),
                            new InstantCommand(),
                            () -> shooter.getState() == Shooter.State.INTAKING))))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.OFF),
                    new InstantCommand(),
                    () -> shooter.getState() == Shooter.State.INTAKING)));

    driver
        .povUp()
        .onTrue(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OUTAKE_GP),
                Shooter.Commands.setState(Shooter.State.OUTAKING),
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING)))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.OFF),
                    new InstantCommand(),
                    () -> shooter.getState() == Shooter.State.OUTAKING)));

    // driver.povLeft().onTrue(ShooterPivot.Commands.setMotionMode(ShooterPivot.State.PODIUM_SHOT));

    // driver.povRight().onTrue(ShooterPivot.Commands.setMotionMode(ShooterPivot.State.FENDER_SHOT));

    driver
        .rightBumper()
        .onTrue(
            Commands.sequence(
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.FENDER_SHOT),
                Shooter.Commands.setState(Shooter.State.FENDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.HOLDING_GP),
                    Shooter.Commands.setState(Shooter.State.OFF),
                    () -> shooter.hasGamePiece()),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.INTAKING)));

    driver
        .rightTrigger(0.3)
        .onTrue(
            Commands.sequence(
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.PODIUM_SHOT),
                Shooter.Commands.setState(Shooter.State.FENDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.HOLDING_GP),
                    Shooter.Commands.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.FENDER_SHOT),
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
        .povDown()
        .whileTrue(
            new RepeatCommand(
                new InstantCommand(
                        () -> {
                          swerveDrive.setMotionMode(MotionMode.HEADING_CONTROLLER);
                          SwerveHeadingController.getInstance()
                              .addToSetpoint(
                                  Rotation2d.fromDegrees(
                                      visionFront.getInputs().horizontalOffsetFromTarget
                                          * Constants.DynamicShooterConstants.heading_kP));
                        })
                    .repeatedly()
                    .until(
                        () ->
                            SwerveHeadingController.getInstance()
                                .atSetpoint(
                                    Constants.DynamicShooterConstants.headingErrorDegree))));
  }

  public void createOperatorBindings() {
    operator
        .x()
        .onTrue(
            Commands.sequence(
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.FENDER_SHOT),
                Shooter.Commands.setState(Shooter.State.FENDER_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.HOLDING_GP),
                    Shooter.Commands.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.FENDER_SHOT),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setModeAndWait(ShooterPivot.State.INTAKING)));

    operator
        .a()
        .onTrue(
            Commands.sequence(
                ShooterPivot.Commands.setMotionMode(ShooterPivot.State.PODIUM_SHOT),
                Shooter.Commands.setState(Shooter.State.PODIUM_SHOT),
                new WaitUntilCommand(() -> shooter.isAtTarget()),
                Intake.Commands.setMotionMode(Intake.State.INTAKE_GP)))
        .onFalse(
            Commands.sequence(
                Intake.Commands.setMotionMode(Intake.State.OFF),
                Commands.either(
                    Shooter.Commands.setState(Shooter.State.HOLDING_GP),
                    Shooter.Commands.setState(Shooter.State.OFF),
                    () -> shooter.getState() == Shooter.State.PODIUM_SHOT),
                new WaitCommand(0.05),
                ShooterPivot.Commands.setModeAndWait(ShooterPivot.State.INTAKING)));

    operator.povUp().onTrue(Elevator.Commands.setState(Elevator.State.MAX_HEIGHT));
    operator.povDown().onTrue(Elevator.Commands.setState(Elevator.State.MIN_HEIGHT));
  }

  public void createAutomaticTriggers() {

    new Trigger(() -> shooter.hasGamePiece())
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> {
                      visionFront.setLEDMode(LEDMode.FORCE_BLINK);
                    }),
                new WaitCommand(2),
                new InstantCommand(
                    () -> {
                      visionFront.setLEDMode(LEDMode.PIPELINE);
                    })));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // RumbleManager.getInstance().periodic();
    mechManager.periodic();

    if (Math.abs(driver.getRightX()) > 0.25) {
      swerveDrive.setMotionMode(MotionMode.FULL_DRIVE);
    }

    // swerveDrive.seed();

    Logger.recordOutput(
        "Filtered CAN Utilization",
        canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    Logger.recordOutput(
        "Memory Usage",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);

    VehicleState.getInstance().updateDynamicPivotAngle(visionFront.estimateDistanceToTag());
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
    seedGyroBasedOnAlliance();
    swerveDrive.seed();
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
  public void teleopPeriodic() {
    RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());
  }

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
    RHRNamedCommands.registerGenericCommands();

    autoChooser.addDefaultOption("ThreePiece", ThreePiece.getAutonomousCommand());
    autoChooser.addOption("SimpleChoreo", SimpleChoreo.getAutonomousCommand());
    autoChooser.addOption("ThreePieceChoreo", new ThreePieceChoreo());
    autoChooser.addOption("Selfish", SelfishAuto.getAutonomousCommand());
    autoChooser.addOption("Week0MobilityChoreo", new Week0MobilityChoreo());
    autoChooser.addOption("NonAmpSide", new NonAmpSide());
  }

  public void updatePreMatchDashboardValues() {
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Has Alliance Color", DriverStation.getAlliance().isPresent());
    SmartDashboard.putBoolean(
        "Front Left Encoder Good",
        swerveDrive.getSwerveModuleStates()[0].angle.getDegrees() != 0.0);
    SmartDashboard.putBoolean(
        "Front Right Encoder Good",
        swerveDrive.getSwerveModuleStates()[1].angle.getDegrees() != 0.0);
    SmartDashboard.putBoolean(
        "Back Left Encoder Good", swerveDrive.getSwerveModuleStates()[2].angle.getDegrees() != 0.0);
    SmartDashboard.putBoolean(
        "Back Right Encoder Good",
        swerveDrive.getSwerveModuleStates()[3].angle.getDegrees() != 0.0);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void seedGyroBasedOnAlliance() {
    Optional<Alliance> checkedAlliance = DriverStation.getAlliance();

    // if we are on blue, we are probably facing towards the blue DS, which is -x.
    // that corresponds to a 180 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Blue) {
      swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
    }

    // if we are on red, we are probably facing towards the red DS, which is +x.
    // that corresponds to a 0 deg heading.
    if (checkedAlliance.isPresent() && checkedAlliance.get() == Alliance.Red) {
      swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
    }
  }

  @Override
  public void driverStationConnected() {
    seedGyroBasedOnAlliance();
    buildAutoChooser();
  }
}
