package frc.robot.subsystems.swerveIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  StatusSignal<Double> yawSignal;

  double offset = 0;

  // Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);

    yawSignal = gyro.getYaw();
    // yawSignal.setUpdateFrequency(50);

    // gyro.reset();
    // gyro.zeroGyroBiasNow();
    // gyro.setYaw(0);

  }

  @Override
  public void updateInputs(
      SwerveInputs inputs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] measuredPositions) {

    BaseStatusSignal.refreshAll(yawSignal);

    // double[] xyz = new double[3];
    // gyro.getAccumGyro(xyz);
    // inputs.gyroYawPosition = xyz[2];

    // inputs.gyroPitchPosition = gyro.getPitch().getValue();
    // inputs.gyroRollPosition = gyro.getRoll().getValue();
    inputs.gyroYawPosition = yawSignal.getValueAsDouble();
    // inputs.gyroYawPosition = gyro.getAccumGyroZ().getValue() - offset;

    // 150 = accumZ - offset
    // offset = accumZ - offset
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    Logger.recordOutput("Reset gyro to", rotation2d.getDegrees());
    // gyro.setYaw(rotation2d.getDegrees());
    gyro.getConfigurator().setYaw(rotation2d.getDegrees());
    // gyro.setYaw(0);
    // offset = gyro.getAccumGyroZ().getValue() - rotation2d.getDegrees();
  }

  public void zeroGyro() {
    Logger.recordOutput("Reset gyro to", 0);
    // gyro.zeroGyroBiasNow();
    // gyro.setYaw(0);
    resetGyro(new Rotation2d());
    // offset = gyro.getAccumGyroZ().getValue();
  }
}
