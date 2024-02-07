package frc.robot.subsystems.shooterIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOVortex implements ShooterIO {
  private static final CANSparkFlex leftMotor =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);
  private static final CANSparkFlex rightMotor =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);

  public ShooterIOVortex() {
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    leftMotor.getPIDController().setP(ShooterConstants.SHOOTER_GAINS.getKP());
    leftMotor.getPIDController().setD(ShooterConstants.SHOOTER_GAINS.getKD());

    rightMotor.getPIDController().setP(ShooterConstants.SHOOTER_GAINS.getKP());
    rightMotor.getPIDController().setD(ShooterConstants.SHOOTER_GAINS.getKD());
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    inputs.leftOutputVoltage = leftMotor.getBusVoltage();
    inputs.rightOutputVoltage = rightMotor.getBusVoltage();

    inputs.leftDrawAmp = leftMotor.getOutputCurrent();
    inputs.rightDrawAmp = rightMotor.getOutputCurrent();

    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();

    inputs.leftPosDeg = Units.rotationsToDegrees(leftMotor.getEncoder().getPosition());
    inputs.rightPosDeg = Units.rotationsToDegrees(rightMotor.getEncoder().getPosition());

    inputs.leftSpeedRPM = leftMotor.getEncoder().getVelocity();
    inputs.rightSpeedRPM = rightMotor.getEncoder().getVelocity();
  }

  @Override
  public void setLeftVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
  }

  @Override
  public void setMotorSetPoint(double setpointRPM) {
    leftMotor.getPIDController().setReference(setpointRPM, ControlType.kVelocity, 0);
    rightMotor.getPIDController().setReference(setpointRPM, ControlType.kVelocity, 0);
  }
}
