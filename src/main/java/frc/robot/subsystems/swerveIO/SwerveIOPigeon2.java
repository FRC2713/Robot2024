package frc.robot.subsystems.swerveIO;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
    gyro.getYaw().setUpdateFrequency(50);
    gyro.getPitch().setUpdateFrequency(50);
    gyro.getRoll().setUpdateFrequency(50);
    gyro.optimizeBusUtilization();

    // gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(
      SwerveInputs inputs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] measuredPositions) {
    inputs.previousgyroPitchPosition = inputs.gyroPitchPosition;

    inputs.gyroPitchPosition = gyro.getPitch().getValue();
    inputs.gyroRollPosition = gyro.getRoll().getValue();
    inputs.gyroYawPosition = gyro.getYaw().getValue(); // gyro faces forwards on the robot
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    Logger.recordOutput("Reset gyro to", rotation2d.getDegrees());
    gyro.setYaw(rotation2d.getDegrees());
  }

  public void zeroGyro() {
    Logger.recordOutput("Reset gyro to", 0);
    // gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }
}
