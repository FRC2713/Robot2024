package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.VehicleState;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.SwerveSubsystem.MotionMode;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SwerveHeadingController;

public class NonAmpSideGP extends RHRFullRoutine {

  private ChoreoTrajectory traj2, traj3;

  public NonAmpSideGP() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Non Amp Side.3"));

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        // Set shooter on always
        Cmds.setState(ShooterState.DYNAMIC_SHOT),

        // First Piece
        ShootingCommands.runPathAndIntakeTowardsGP(traj1),
        new InstantCommand(
            () -> {
              VehicleState.getInstance().resetClosestGP();
            }),
        Commands.either(
            Commands.sequence(
                Cmds.setState(MotionMode.ALIGN_TO_TAG),
                new WaitUntilCommand(() -> SwerveHeadingController.getInstance().atSetpoint(0.3)),
                ShootingCommands.runShooterAndPivot(
                    ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
                RedHawkUtil.logShot()),
            Commands.none(),
            Robot.shooter::hasGamePiece),
        Cmds.setState(MotionMode.TRAJECTORY),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Second Piece
        ShootingCommands.runPathAndIntake(traj2),
        new WaitCommand(0.7),
        Commands.either(
            Commands.sequence(
                Cmds.setState(MotionMode.ALIGN_TO_TAG),
                new WaitUntilCommand(() -> SwerveHeadingController.getInstance().atSetpoint(0.3)),
                ShootingCommands.runShooterAndPivot(
                    ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
                RedHawkUtil.logShot()),
            Commands.none(),
            Robot.shooter::hasGamePiece),
        Cmds.setState(MotionMode.TRAJECTORY),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Reset everything for teleop
        Cmds.setState(ShooterState.OFF),
        Cmds.setState(Intake.State.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Go for 3
        ShootingCommands.runPathAndIntake(traj3));
  }
}
