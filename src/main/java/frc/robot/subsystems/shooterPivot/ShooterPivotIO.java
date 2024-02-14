package frc.robot.subsystems.shooterPivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

  @AutoLog
  public static class ShooterPivotInputs {
    public double outputVoltage = 0.0;
    public double angleDegreesMotor = 0.0;
    public double velocityDegreesPerSecondMotor = 0.0;
    public double tempCelcius = 0.0;
    public double currentDraw = 0.0;
    public double absoluteEncoderAdjustedAngle = 0.0;
    public double absoluteEncoderRawPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public double targetAngle = 0.0;
  }

  public void updateInputs(ShooterPivotInputs inputs);

  public void reseedPosition(double angleDeg);

  public void setVoltage(double volts);

  public void setTargetPosition(double angleDeg);
}
