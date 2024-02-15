package frc.robot.subsystems.shooterPivot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class ShooterPivotIOSparks implements ShooterPivotIO {

  CANSparkMax spark;
  SparkAbsoluteEncoder throughBoreEncoder;

  public ShooterPivotIOSparks() {
    spark = new CANSparkMax(Constants.RobotMap.PIVOT_ID, MotorType.kBrushless);
    spark.restoreFactoryDefaults();

    spark.getEncoder().setPositionConversionFactor(1 / ShooterPivotConstants.GEARING);
    spark.getEncoder().setVelocityConversionFactor(1 / ShooterPivotConstants.GEARING);

    throughBoreEncoder = spark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    spark.getPIDController().setFeedbackDevice(throughBoreEncoder);
    spark.getPIDController().setP(0.05);

    throughBoreEncoder.setPositionConversionFactor(360.0 / (80.0 / 20.0));
    throughBoreEncoder.setVelocityConversionFactor(360.0 / (80.0 / 20.0));

    spark.setSmartCurrentLimit(20);
    spark.setIdleMode(IdleMode.kBrake);

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 20);
            put(PeriodicFrame.kStatus1, 20);
            put(PeriodicFrame.kStatus2, 20);
            put(PeriodicFrame.kStatus3, 20);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 20);
            put(PeriodicFrame.kStatus6, 20);
          }
        },
        spark);

    spark.setInverted(false);
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    inputs.angleDegreesMotor = Units.rotationsToDegrees(spark.getEncoder().getPosition());
    inputs.velocityDegreesPerSecondMotor =
        Units.radiansToDegrees(
            Units.rotationsPerMinuteToRadiansPerSecond(spark.getEncoder().getVelocity()));
    inputs.tempCelcius = spark.getMotorTemperature();
    inputs.outputVoltage = spark.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.absoluteEncoderRawPosition = throughBoreEncoder.getPosition();
    inputs.absoluteEncoderAdjustedAngle =
        throughBoreEncoder.getPosition() + ShooterPivotConstants.OFFSET;
    inputs.absoluteEncoderVelocity = throughBoreEncoder.getVelocity();
    inputs.currentDraw = spark.getOutputCurrent();
  }

  @Override
  public void reseedPosition(double angleDeg) {
    // double trueAngle = angleDeg + Constants.ShooterPivotConstants.OFFSET;
    // spark.getEncoder().setPosition(trueAngle);
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void setTargetPosition(double angleDeg) {
    angleDeg = angleDeg - Constants.ShooterPivotConstants.OFFSET;
    Logger.recordOutput("ShooterPivot/Offset Target", angleDeg);
    spark.getPIDController().setReference(angleDeg, ControlType.kPosition);
  }
}
