package frc.robot.subsystems.shooterPivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

  @AutoLog
  public static class ShooterPivotInputs {
    public double outputVoltage = 0.0;
    public double angleDegreesOne = 0.0;
    public double velocityDegreesPerSecondOne = 0.0;
    public double tempCelciusOne = 0.0;
    public double currentDrawOne = 0.0;
    public double absoluteEncoderVolts = 0.0;
    public double absoluteEncoderAdjustedAngle = 0.0;
    public double targetAngle = 0.0;
  }

  public void updateInputs(ShooterPivotInputs inputs);

  public void reseedPosition(double angleDeg);

  public void setVoltage(double volts);

  public void setTargetPosition(double angleDeg);
}
