package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(Constants.RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.RobotMap.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    right.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltageLeft =
        MathUtil.clamp(left.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesLeft = left.getEncoder().getPosition();
    inputs.velocityInchesPerSecondLeft = left.getEncoder().getVelocity();
    inputs.tempCelsiusLeft = left.getMotorTemperature();
    inputs.currentDrawAmpsLeft = left.getOutputCurrent();
    inputs.outputVoltageRight =
        MathUtil.clamp(right.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesRight = right.getEncoder().getPosition();
    inputs.velocityInchesPerSecondRight = right.getEncoder().getVelocity();
    inputs.tempCelsiusRight = right.getMotorTemperature();
    inputs.currentDrawAmpsRight = right.getOutputCurrent();
  }

  @Override
  public void resetEncoders() {
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);
    ;
  }

  @Override
  public boolean shouldApplyFF() {
    return true;
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
    ;
  }
}