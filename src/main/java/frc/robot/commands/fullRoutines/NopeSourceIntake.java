package frc.robot.commands.fullRoutines;

import com.choreo.lib.Choreo;
import frc.robot.commands.Cmds;
import frc.robot.commands.RHRFullRoutine;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.shooterIO.Shooter;
import frc.robot.subsystems.shooterIO.Shooter.FeederState;
import frc.robot.subsystems.shooterIO.Shooter.ShooterState;
import frc.robot.subsystems.shooterPivot.ShooterPivot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.RedHawkUtil;

public class NopeSourceIntake extends RHRFullRoutine {
  public NopeSourceIntake() {
    traj1 = RedHawkUtil.maybeFlip(Choreo.getTrajectory("NopeSourceIntake"));

    addCommands(
        SwerveSubsystem.Commands.resetOdometry(traj1),

        // Preload
        ShootingCommands.runShooterPivot(ShooterPivot.State.FENDER_SHOT),
        ShootingCommands.runShooter(Shooter.ShooterState.NO_DIFFERENTIAL_SHOT),
        RedHawkUtil.logShot(),
        Cmds.setState(ShooterPivot.State.INTAKING),
        Cmds.setState(ShooterState.GO_MY_WAY),
        Cmds.setState(FeederState.FEED_CONTINUOUS),
        Cmds.setState(Intake.State.INTAKE_GP),

        // Lets go!
        ShootingCommands.runPath(traj1));
  }
}
