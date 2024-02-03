package frc.robot.subsystems.intakeIO;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparks implements IntakeIO {

  private TimeOfFlight sensor;
  private CANSparkMax leftMotor, rightMotor;

  public IntakeIOSparks() {
    this.sensor = new TimeOfFlight(Constants.RobotMap.INTAKE_TOF_SENSOR_ID);
    this.sensor.setRangingMode(RangingMode.Medium, 24);
    this.leftMotor =
        new CANSparkMax(Constants.RobotMap.INTAKE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    this.rightMotor =
        new CANSparkMax(Constants.RobotMap.INTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);

    rightMotor.setInverted(true);
    leftMotor.setInverted(false);
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    leftMotor.setSmartCurrentLimit(currentLimit);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.leftOutputVoltage = MathUtil.clamp(leftMotor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.leftIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(leftMotor.getEncoder().getVelocity()) > 0.005;
    inputs.leftVelocityRPM = leftMotor.getEncoder().getVelocity();
    inputs.leftTempCelcius = leftMotor.getMotorTemperature();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
    inputs.leftPositionRad = leftMotor.getEncoder().getPosition()*Math.PI*2;

    inputs.rightOutputVoltage = MathUtil.clamp(rightMotor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.rightIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(rightMotor.getEncoder().getVelocity()) > 0.005;
    inputs.rightVelocityRPM = rightMotor.getEncoder().getVelocity();
    inputs.rightTempCelcius = rightMotor.getMotorTemperature();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    inputs.rightPositionRad = leftMotor.getEncoder().getPosition()*Math.PI*2;

    inputs.sensorRange = sensor.getRange();
    inputs.sensorStatus = sensor.getStatus().toString();
  }

  @Override
  public boolean hasGamepiece() {
    return (sensor.getRange() < Constants.IntakeConstants.SENSOR_THRESHOLD);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }
}
