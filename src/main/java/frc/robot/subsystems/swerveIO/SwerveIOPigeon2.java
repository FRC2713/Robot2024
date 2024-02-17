package frc.robot.subsystems.swerveIO;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  double offset = 0;

  // Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);

    // gyro.reset();
    // gyro.getYaw().setUpdateFrequency(100);
    // gyro.getPitch().setUpdateFrequency(100);
    // gyro.getRoll().setUpdateFrequency(100);
    // gyro.optimizeBusUtilization();

    // gyro.zeroGyroBiasNow();
    // gyro.setYaw(0);

  }

  @Override
  public void updateInputs(
      SwerveInputs inputs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] measuredPositions) {

    // double[] xyz = new double[3];
    // gyro.getAccumGyro(xyz);
    // inputs.gyroYawPosition = xyz[2];

    // inputs.gyroPitchPosition = gyro.getPitch();
    // inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getAccumGyroZ().getValue() - offset;

    // 150 = accumZ - offset
    // offset = accumZ - offset
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    Logger.recordOutput("Reset gyro to", rotation2d.getDegrees());
    // gyro.setYaw(rotation2d.getDegrees());
    // gyro.setYaw(0);
    offset = gyro.getAccumGyroZ().getValue() - rotation2d.getDegrees();
  }

  public void zeroGyro() {
    Logger.recordOutput("Reset gyro to", 0);
    // gyro.zeroGyroBiasNow();
    // gyro.setYaw(0);
    offset = gyro.getAccumGyroZ().getValue();
  }
}
