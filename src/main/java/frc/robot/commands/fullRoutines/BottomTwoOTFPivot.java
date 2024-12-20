package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
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

public class BottomTwoOTFPivot extends RHRFullRoutine {
  private ChoreoTrajectory traj2, traj3;

  public BottomTwoOTFPivot() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.1"));
    traj2 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.2"));
    traj3 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("Bottom Two.3"));

    RedHawkUtil.maybeFlipLog(traj1);

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        // Set shooter on always
        Cmds.setState(ShooterState.DIFFERENTIAL_SHOT),

        // First Piece
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj1, ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
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
        ShootingCommands.runPathIntakeWaitTillHasGPThenPrepShooterPivotAndShooter(
            traj2, ShooterState.DIFFERENTIAL_SHOT, ShooterPivot.State.POSE_AIM),
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
        Cmds.setState(Shooter.ShooterState.OFF),
        Cmds.setState(Intake.State.OFF),
        Cmds.setState(ShooterPivot.State.INTAKING),

        // Clear out
        ShootingCommands.runPathAndIntake(traj3));
  }
}
