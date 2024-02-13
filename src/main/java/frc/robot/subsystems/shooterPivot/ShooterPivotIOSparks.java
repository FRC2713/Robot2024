package frc.robot.subsystems.shooterPivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.rhr.RHRFeedForward;
import frc.robot.rhr.RHRPIDFFController;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;

public class ShooterPivotIOSparks implements ShooterPivotIO {

  CANSparkMax spark;
  SparkAnalogSensor analogSensor;

  private double targetAngle;

  private RHRPIDFFController motorController;

  private RHRFeedForward feedforward;

  public ShooterPivotIOSparks() {
    spark = new CANSparkMax(Constants.RobotMap.PIVOT_ID, MotorType.kBrushless);
    spark.restoreFactoryDefaults();

    analogSensor = spark.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    spark.getPIDController().setFeedbackDevice(analogSensor);

    spark.setSmartCurrentLimit(20);

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 20);
            put(PeriodicFrame.kStatus1, 20);
            put(PeriodicFrame.kStatus2, 20);
            put(PeriodicFrame.kStatus3, 65535);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 20);
            put(PeriodicFrame.kStatus6, 20);
          }
        },
        spark);

    spark.setIdleMode(IdleMode.kBrake);
    spark.setInverted(false);

    motorController = Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createRHRController();
    feedforward = Constants.ShooterPivotConstants.SHOOTER_PIVOT_GAINS.createRHRFeedForward();
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    inputs.absoluteEncoderAdjustedAngle =
        Units.rotationsToDegrees(spark.getEncoder().getPosition());

    inputs.angleDegreesOne = inputs.absoluteEncoderAdjustedAngle;

    inputs.velocityDegreesPerSecondOne =
        Units.radiansToDegrees(
            Units.rotationsPerMinuteToRadiansPerSecond(spark.getEncoder().getVelocity()));

    inputs.tempCelciusOne = spark.getMotorTemperature();

    inputs.currentDrawOne = spark.getOutputCurrent();

    inputs.outputVoltage = spark.getBusVoltage();
  }

  @Override
  public void reseedPosition(double angleDeg) {
    double trueAngle = angleDeg - Constants.ShooterPivotConstants.OFFSET;
    spark.getEncoder().setPosition(trueAngle);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setTargetPosition(double angleDeg) {
    this.targetAngle = angleDeg;
    this.spark.getPIDController().setReference(angleDeg, ControlType.kPosition);
  }
}
