package frc.robot.subsystems.shooterPivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;

public class ShooterPivotIOSparks implements ShooterPivotIO {

  CANSparkMax spark;
  SparkAnalogSensor analogSensor;

  public ShooterPivotIOSparks() {
    spark = new CANSparkMax(0, MotorType.kBrushless);
    analogSensor = spark.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

    spark.restoreFactoryDefaults();
    spark.getPIDController().setFeedbackDevice(analogSensor);

    spark.setSmartCurrentLimit(20);
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
        spark);

    spark.setIdleMode(IdleMode.kBrake);
    spark.setInverted(false);
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void reseedPosition(double angleDeg) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reseedPosition'");
  }

  @Override
  public void setVoltage(double volts) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void setTargetPosition(double angleDeg) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
  }
}
