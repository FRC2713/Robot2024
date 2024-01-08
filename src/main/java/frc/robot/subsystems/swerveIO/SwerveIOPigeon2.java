package frc.robot.subsystems.swerveIO;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.RobotMap;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
    // RedHawkUtil.configureDefaultPigeon2(gyro);
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html

    RedHawkUtil.configurePigeonStatusFrames(
        gyro,
        new HashMap<>() {
          {
            put(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 255);
            put(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 255);
            put(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
            put(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 255);
            put(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 255);
            put(PigeonIMU_StatusFrame.CondStatus_1_General, 255);
            put(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 255);
            put(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 255);
            put(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 255);
            put(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20);
            put(PigeonIMU_StatusFrame.RawStatus_4_Mag, 255);
          }
        });

    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(
      SwerveInputs inputs,
      SwerveDriveKinematics kinematics,
      SwerveModulePosition[] measuredPositions) {
    inputs.previousgyroPitchPosition = inputs.gyroPitchPosition;
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getYaw(); // gyro faces forwards on the robot
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    Logger.getInstance().recordOutput("Reset gyro to", rotation2d.getDegrees());
    gyro.setYaw(rotation2d.getDegrees());
  }

  public void zeroGyro() {
    Logger.getInstance().recordOutput("Reset gyro to", 0);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }
}
