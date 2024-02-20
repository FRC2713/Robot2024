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

    public double feederOutputVolts = 0.0;
    public double feederStatorCurrentAmps = 0.0;
    public double feederSupplyCurrentAmps = 0.0;
    public double feederVelocityRPM = 0.0;

    public double sensorVoltage = 0.0;
  }

  public void updateInputs(ShooterInputsAutoLogged inputs);

  public void setMotorSetPoint(double leftRPM, double rightRPM);

  public void setFeederVolts(double volts);
}
