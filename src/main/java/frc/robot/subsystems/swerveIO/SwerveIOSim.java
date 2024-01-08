package frc.robot.subsystems.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class SwerveIOSim implements SwerveIO {

  private Rotation2d currentRotation = new Rotation2d();
  private SwerveModulePosition[] lastPositions;

  @Override
  public void updateInputs(
      SwerveInputs inputs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] measuredPositions) {
    if (lastPositions == null) {
      lastPositions = measuredPositions;
    }

    SwerveModulePosition[] deltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      deltas[i] = RedHawkUtil.moduleDelta(measuredPositions[i], lastPositions[i]);
      Logger.getInstance().recordOutput("Swerve/Sim/" + i + "-pos", deltas[i].distanceMeters);
      Logger.getInstance().recordOutput("Swerve/Sim/" + i + "-rot", deltas[i].angle.getDegrees());
      Logger.getInstance()
          .recordOutput("Swerve/Sim/" + i + "-last-pos", lastPositions[i].distanceMeters);
      Logger.getInstance()
          .recordOutput("Swerve/Sim/" + i + "-last-rot", lastPositions[i].angle.getDegrees());

      Logger.getInstance()
          .recordOutput("Swerve/Sim/" + i + "-meas-pos", measuredPositions[i].distanceMeters);
      Logger.getInstance()
          .recordOutput("Swerve/Sim/" + i + "-meas-rot", measuredPositions[i].angle.getDegrees());
    }

    currentRotation =
        currentRotation.plus(Rotation2d.fromRadians(kinematics.toTwist2d(deltas).dtheta));

    lastPositions = measuredPositions;

    inputs.gyroCompassHeading = 0;
    inputs.gyroPitchPosition = 0;
    inputs.previousgyroPitchPosition = 0;
    inputs.gyroRollPosition = 0;
    inputs.gyroYawPosition = currentRotation.getDegrees();
  }

  public void zeroGyro() {
    currentRotation = new Rotation2d();
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    currentRotation = rotation2d;
  }
}
