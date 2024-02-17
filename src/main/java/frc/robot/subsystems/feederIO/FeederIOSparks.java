package frc.robot.subsystems.feederIO;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap;

public class FeederIOSparks implements FeederIO {
  CANSparkMax motor;
  double setpoint = 0;
  private TimeOfFlight sensor;

  public FeederIOSparks() {
    motor = new CANSparkMax(RobotMap.FEEDER_CAN_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
    motor.setInverted(true);

    // motor.getPIDController().setP(FeederConstants.FEEDER_GAINS.getKP());
    // motor.getPIDController().setD(FeederConstants.FEEDER_GAINS.getKD());
    sensor = new TimeOfFlight(71);
  }

  @Override
  public void setSetpoint(double setpointRPM) {
    motor.getPIDController().setReference(setpointRPM, ControlType.kVelocity, 0);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public boolean atTarget() {
    return Math.abs(motor.getEncoder().getVelocity() - setpoint) < 0.1;
  }

  @Override
  public void updateInputs(FeederInputsAutoLogged inputs) {
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.outputVoltage = motor.getAppliedOutput();
    inputs.isOn = Math.abs(motor.getEncoder().getVelocity()) > 0.05;
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.positionDeg = Units.rotationsToDegrees(motor.getEncoder().getPosition());
  }

  @Override
  public boolean hasGamepiece() {
    // return (sensor.getRange() < Constants.FeederConstants.SENSOR_THRESHOLD);
    return false;
  }
}
