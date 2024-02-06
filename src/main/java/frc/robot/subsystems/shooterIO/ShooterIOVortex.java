package frc.robot.subsystems.shooterIO;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterIOVortex implements ShooterIO {

  /*CANSparkFlex leftFlyWheel;
  CANSparkFlex rightFlyWheel;

  RHRPIDFFController leftFlyWheelController;
  RHRPIDFFController rightFlyWheelController;*/

  private static final CANSparkFlex leftFlyWheel =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);
  private static final CANSparkFlex rightFlyWheel =
      new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.leftOutputVoltage = leftFlyWheel.getBusVoltage();
    inputs.rightOutputVoltage = rightFlyWheel.getBusVoltage();

    inputs.leftFLyWheelDrawAmp = leftFlyWheel.getOutputCurrent();
    inputs.rightFLyWheelDrawAmp = rightFlyWheel.getOutputCurrent();

    inputs.leftTempCelcius = leftFlyWheel.getMotorTemperature();
    inputs.rightTempCelcius = rightFlyWheel.getMotorTemperature();
  }

  @Override
  public void setLeftVoltage(double voltage) {
    leftFlyWheel.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    rightFlyWheel.setVoltage(voltage);
  }

  /*
  public void setLeftMotorRPMSetPoint(double rPM) {
    leftFlyWheelController.setSetpoint(rPM);
  }

  public void setRightMotorRPMSetPoint(double rPM) {
    rightFlyWheelController.setSetpoint(rPM);
  }*/

}
