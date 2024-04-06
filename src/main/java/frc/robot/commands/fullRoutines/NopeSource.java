package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.FeederState;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.util.RedHawkUtil;

public class NopeSource extends RHRFullRoutine {
  public NopeSource() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("NopeSource"));

    addCommands(
        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),

        Cmds.setState(Intake.State.INTAKE_GP),

        new ParallelCommandGroup(
             ShootingCommands.runPath(traj1),

            new RepeatCommand(
                Commands.sequence(
                    new WaitUntilCommand(() -> Robot.shooter.hasGamePiece()),
                    Cmds.setState(Intake.State.OFF),
                    Cmds.setState(ShooterState.DIFFERENTIAL_SHOT),
                    new WaitCommand(0.2),
                    Cmds.setState(FeederState.FEED_SHOT),
                    new WaitUntilCommand(() -> !Robot.shooter.hasGamePiece()),
                    new WaitCommand(0.1),
                    Cmds.setState(Intake.State.INTAKE_GP)))
           ));
  }
}
