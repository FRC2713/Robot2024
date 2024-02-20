package frc.robot.commands.otf;

import static frc.robot.util.RedHawkUtil.Translation3dTo2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SwerveHeadingController;
import org.littletonrobotics.junction.Logger;

public class RotateScore extends SequentialCommandGroup {
  private static Translation3d speakerLoc =
      RedHawkUtil.Reflections.reflectIfRed(new Translation3d(0.695, 5.552, 2.11));

  public static Rotation2d getOptimalAngle(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));

    var optimalAngle = Math.acos((position.getX() - speakerLoc.getX()) / distance);
    if (position.getY() < speakerLoc.getY()) {
      optimalAngle *= -1;
    }
    Logger.recordOutput(
        "OTF/Optimal Angle", new Pose2d(position.getTranslation(), new Rotation2d(optimalAngle)));
    return RedHawkUtil.Reflections.reflectIfRed(new Rotation2d(optimalAngle));
  }

  public static Command optimalShoot() {
    return new InstantCommand(
        () -> {
          var optimalHeight = getOptimalElevatorHeightMetres(Robot.swerveDrive.getUsablePose());
          Logger.recordOutput("OTF/Optimal Height", optimalHeight);
          Elevator.Commands.setToHeightAndWait(optimalHeight).schedule();
          var optimalShooterAngle =
              getOptimalShooterAngle(Robot.swerveDrive.getUsablePose(), optimalHeight);
          Logger.recordOutput("OTF/Optimal Shooter Angle", optimalShooterAngle.getDegrees() + 90);
          // Robot.shooterPivot.setTargetAngle(optimalShooterAngle.getDegrees());
          Rotation2d optimalRotation =
              RotateScore.getOptimalAngle(Robot.swerveDrive.getUsablePose());
          SwerveHeadingController.getInstance().setSetpoint(optimalRotation);
        });
  }

  public static Rotation2d getOptimalShooterAngle(Pose2d position, double elevatorHeight) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));
    Logger.recordOutput(
        "OTF/Offset Speaker Height",
        speakerLoc.getZ()
            - Constants.ElevatorConstants.FLOOR_TO_ELEVATOR_BASE_METRES
            - Units.inchesToMeters(elevatorHeight));
    Logger.recordOutput("OTF/DIST", distance);
    return new Rotation2d(
        Math.atan(
            (speakerLoc.getZ()
                    - Constants.ElevatorConstants.FLOOR_TO_ELEVATOR_BASE_METRES
                    - elevatorHeight)
                / distance));
  }

  private static InterpolatingTreeMap<Double, Double> heights =
      new InterpolatingTreeMap<>() {
        {
          // Dist (metres), Height (in)
          put(0., 0.);
          put(1., 5.);
          put(2., 10.);
          put(3., 15.);
          put(4., 20.);
        }
      };

  public static double getOptimalElevatorHeightMetres(Pose2d position) {
    var distance = position.getTranslation().getDistance(Translation3dTo2d(speakerLoc));
    return heights.get(distance);
  }
}
