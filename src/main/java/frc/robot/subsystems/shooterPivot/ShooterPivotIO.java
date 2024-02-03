package frc.robot.subsystems.shooterPivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

  @AutoLog
  public static class ShooterPivotInputs {
    public double outputVoltage = 0.0;

    public double angleDegreesOne = 0.0;
    public double angleDegreesTwo = 0.0;
    public double angleDegreesRange = 0.0;

    public double velocityDegreesPerSecondOne = 0.0;
    public double velocityDegreesPerSecondTwo = 0.0;
    public double velocityDegreesPerSecondRange = 0.0;

    public double tempCelciusOne = 0.0;
    public double tempCelciusTwo = 0.0;

    public double currentDrawOne = 0.0;
    public double currentDrawTwo = 0.0;

    public double absoluteEncoderVolts = 0.0;
    public double absoluteEncoderAdjustedAngle = 0.0;

    public boolean limSwitch = false;
  }

  public void reseed(double absoluteEncoderVolts);

  public void updateInputs(ShooterPivotInputs inputs);

  public void setVoltage(double volts);

  public void setPosition(double angleDeg);

  public void setCurrentLimit(int currentLimit);

  public double getAngleDegrees();
}
