package frc.robot.subsystems.shooterPivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

  @AutoLog
  public static class ShooterPivotInputs {
    public double outputVoltageLeft = 0.0;
    public double angleDegreesLeft = 0.0;
    public double velocityDegreesPerSecondLeft = 0.0;
    public double tempCelciusLeft = 0.0;
    public double currentDrawAmpsLeft = 0.0;

    public double outputVoltageRight = 0.0;
    public double angleDegreesRight = 0.0;
    public double velocityDegreesPerSecondRight = 0.0;
    public double tempCelciusRight = 0.0;
    public double currentDrawAmpsRight = 0.0;

    public double absoluteEncoderVolts = 0.0;
    public double absoluteEncoderAdjustedAngle = 0.0;
  }

  public void updateInputs(ShooterPivotInputs inputs);

  public void setTargetAngle(double degrees, double arbFeedForward);
}
