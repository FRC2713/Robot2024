package frc.robot.subsystems.shooterIO;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.rhr.RHRPIDFFController;

public class ShooterIOVortex implements ShooterIO {

  /*CANSparkFlex leftFlyWheel;
  CANSparkFlex rightFlyWheel;

  RHRPIDFFController leftFlyWheelController;
  RHRPIDFFController rightFlyWheelController;*/

  private static final CANSparkFlex leftFlyWheel =
        new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);
    private static final CANSparkFlex rightFlyWheel =
        new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID, MotorType.kBrushless);
  private static final RHRPIDFFController leftFlyWheelController = Constants.ShooterConstants.MOTOR_GAINS;
  private static final RHRPIDFFController rightFlyWheelController = Constants.ShooterConstants.MOTOR_GAINS;

  @Override
  public void updateInputs(ShooterInputs inputs)
  {
    inputs.leftOutputVoltage = leftFlyWheel.getBusVoltage();
    inputs.rightOutputVoltage = rightFlyWheel.getBusVoltage();

    inputs.leftFLyWheelDrawAmp = leftFlyWheel.getOutputCurrent();
    inputs.rightFLyWheelDrawAmp = rightFlyWheel.getOutputCurrent();

    inputs.leftTempCelcius = leftFlyWheel.getMotorTemperature();
    inputs.rightTempCelcius = rightFlyWheel.getMotorTemperature();


  }

  @Override
  public void setLeftVoltage(double voltage)
  {
    leftFlyWheel.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage)
  {
    rightFlyWheel.setVoltage(voltage);
  }

  @Override
  public void setLeftMotorRPMSetPoint(double rPM) {
    leftFlyWheelController.setSetpoint(rPM);
  }

  @Override
  public void setRightMotorRPMSetPoint(double rPM) {
    rightFlyWheelController.setSetpoint(rPM);
  }

  @Override
  public void effort(ShooterInputs inputs)
  {
    double effortRight = rightFlyWheelController.calculate(inputs.rightFlyWheelSpeedRPM,rightFlyWheelController.getSetpoint());
    double effortLeft = rightFlyWheelController.calculate(inputs.leftFlyWheelSpeedRPM,leftFlyWheelController.getSetpoint());

    effortLeft = MathUtil.clamp(effortLeft,-12,12);
    effortRight = MathUtil.clamp(effortRight, -12, 12);

    this.setLeftVoltage(effortLeft);
    this.setRightVoltage(effortRight);
    
  }

}
