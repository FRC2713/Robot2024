package frc.robot.subsystems.feederIO;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotMap;

public class FeederIOSparks implements FeederIO {
  CANSparkMax motor;
  // double setpoint = 0;

  public FeederIOSparks() {
    motor = new CANSparkMax(RobotMap.FEEDER_CAN_ID, MotorType.kBrushless);
  }

  // @Override
  // public void setSetpoint(double setpointRPM) {
  //     setpoint = setpointRPM;
  // }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  // @Override
  // public boolean atTarget() {
  //     return Math.abs(motor.getEncoder().getVelocity() - setpoint) < 0.1;
  // }

  @Override
  public void updateInputs(FeederInputsAutoLogged inputs) {
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.outputVoltage = motor.getAppliedOutput();
    inputs.isOn = Math.abs(motor.getEncoder().getVelocity()) > 0.05;
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.positionDeg = Units.rotationsToDegrees(motor.getEncoder().getPosition());
  }
}
