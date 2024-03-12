package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(Constants.RobotMap.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.RobotMap.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);

    left.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    right.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);

    left.getEncoder().setPositionConversionFactor(1 / 20.0 * Math.PI * 1.7567);
    right.getEncoder().setPositionConversionFactor(1 / 20.0 * Math.PI * 1.7567);

    left.getPIDController().setP(ElevatorConstants.ELEVATOR_GAINS.getKP());
    left.getPIDController().setD(ElevatorConstants.ELEVATOR_GAINS.getKD());

    right.getPIDController().setP(ElevatorConstants.ELEVATOR_GAINS.getKP());
    right.getPIDController().setD(ElevatorConstants.ELEVATOR_GAINS.getKD());

    for (int i = 0; i < 30; i++) {
      left.setInverted(true);
      right.setInverted(true);
    }
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
  public void reset() {
    // left.getEncoder().setPosition(0);
    // right.getEncoder().setPosition(0);

    // left.getEncoder().setPositionConversionFactor(0);
  }

  @Override
  public boolean shouldApplyFF() {
    return true;
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
  }

  @Override
  public void setTargetHeight(double heightInches) {
    left.getPIDController()
        .setReference(
            heightInches,
            ControlType.kPosition,
            0,
            ElevatorConstants.ELEVATOR_GAINS.getKG(),
            ArbFFUnits.kVoltage);
    right
        .getPIDController()
        .setReference(
            heightInches,
            ControlType.kPosition,
            0,
            ElevatorConstants.ELEVATOR_GAINS.getKG(),
            ArbFFUnits.kVoltage);
  }
}
