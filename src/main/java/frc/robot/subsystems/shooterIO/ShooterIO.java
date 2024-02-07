package frc.robot.subsystems.shooterIO;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public double leftOutputVoltage = 0.0;
    public double rightOutputVoltage = 0.0;
    public double leftSpeedRPM = 0.0;
    public double rightSpeedRPM = 0.0;
    public double rightPosDeg = 0.0;
    public double leftPosDeg = 0.0;
    public double leftDrawAmp = 0;
    public double rightDrawAmp = 0;
    public double leftTempCelcius = 0.0;
    public double rightTempCelcius = 0.0;
  }

  public void updateInputs(ShooterInputsAutoLogged inputs);

  public void setLeftVoltage(double voltage);

  public void setRightVoltage(double voltage);

  public void setMotorSetPoint(double setpointRPM);
}
