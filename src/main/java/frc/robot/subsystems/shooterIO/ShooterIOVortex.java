package frc.robot.subsystems.shooterIO;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.rhr.RHRPIDFFController;

public class ShooterIOVortex implements ShooterIO {


    CANSparkFlex leftFlyWheel;
    CANSparkFlex rightFlyWheel;

    RHRPIDFFController leftFlyWheelController;
    RHRPIDFFController rightFlyWheelController;

    public ShooterIOVortex()
    {
        leftFlyWheel = new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID,MotorType.kBrushless);
        rightFlyWheel = new CANSparkFlex(Constants.RobotMap.SHOOTER_LEFT_FLYWHEEL_ID,MotorType.kBrushless);

        leftFlyWheelController = Constants.ShooterConstants.MOTOR_GAINS;
        rightFlyWheelController = Constants.ShooterConstants.MOTOR_GAINS;
    }

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
    public void setLeftVoltage(double voltage)
    {

    }

  @Override
  public void setRightVoltage(double voltage)
  {

  }

  @Override
  public void setLeftMotorRPMSetPoint(double rPM) {
  }

  @Override
  public void setRightMotorRPMSetPoint(double rPM) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setRightMotorRPMSetPoint'");
  }
}
