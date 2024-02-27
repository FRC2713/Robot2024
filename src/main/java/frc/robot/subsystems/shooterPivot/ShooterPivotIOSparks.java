package frc.robot.subsystems.shooterPivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;

public class ShooterPivotIOSparks implements ShooterPivotIO {

  CANSparkFlex left, right;
  SparkAbsoluteEncoder throughBore;

  public ShooterPivotIOSparks() {
    left = new CANSparkFlex(Constants.RobotMap.PIVOT_LEFT_CAN_ID, MotorType.kBrushless);
    right = new CANSparkFlex(Constants.RobotMap.PIVOT_RIGHT_CAN_ID, MotorType.kBrushless);

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    throughBore = left.getAbsoluteEncoder();

    right.getEncoder().setPositionConversionFactor(1.0 / 90.0 * 360.0);
    left.getEncoder().setPositionConversionFactor(1.0 / 90.0 * 360.0);

    left.getEncoder().setPosition(54.255);
    right.getEncoder().setPosition(54.255);

    left.setSmartCurrentLimit(10);
    right.setSmartCurrentLimit(10);

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 60);
            put(PeriodicFrame.kStatus1, 40);
            put(PeriodicFrame.kStatus2, 40);
            put(PeriodicFrame.kStatus3, 65535);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 20);
            put(PeriodicFrame.kStatus6, 20);
          }
        },
        left,
        right);

    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);
    left.setInverted(true);
    right.follow(left, true);

    ShooterPivotConstants.SHOOTER_PIVOT_GAINS.applyTo(left.getPIDController());
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    inputs.angleDegreesLeft = left.getEncoder().getPosition();

    inputs.velocityDegreesPerSecondLeft =
        Units.radiansToDegrees(
            Units.rotationsPerMinuteToRadiansPerSecond(left.getEncoder().getVelocity()));
    inputs.tempCelciusLeft = left.getMotorTemperature();
    inputs.currentDrawAmpsLeft = left.getOutputCurrent();
    inputs.outputVoltageLeft = left.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.angleDegreesRight = right.getEncoder().getPosition();

    inputs.velocityDegreesPerSecondRight =
        Units.radiansToDegrees(
            Units.rotationsPerMinuteToRadiansPerSecond(right.getEncoder().getVelocity()));
    inputs.tempCelciusRight = right.getMotorTemperature();
    inputs.currentDrawAmpsRight = right.getOutputCurrent();
    inputs.outputVoltageRight = right.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.absoluteEncoderAdjustedAngle = throughBore.getPosition();
  }

  @Override
  public void setTargetAngle(double degrees) {
    left.getPIDController().setReference(degrees, ControlType.kPosition, 0, 0, ArbFFUnits.kVoltage);
  }
}
