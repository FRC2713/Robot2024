package frc.robot.subsystems.shooterIO;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public double leftOutputVoltage = 0.0;
    public double rightOutputVoltage = 0.0;
    public double leftDutyCycle = 0.0;
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

    public int laserCanStatus = 0;
    public int laserCanDistanceMM = 0;
    public int laserCanAmbientLightLevel = 0;
    public boolean LSTripped = false;
  }

  public void updateInputs(ShooterInputsAutoLogged inputs, Shooter.State state);

  public void setMotorSetPoint(double leftRPM, double rightRPM);

  public void setFeederVolts(double volts);

  public void setShooterVolts(double lVolts, double rVolts);

  public default void setDisableOnLimitSwitch(boolean val) {}
  ;
}
