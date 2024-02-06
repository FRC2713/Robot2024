package frc.robot.subsystems.feederIO;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederInputs {
    public double outputVoltage = 0.0;
    public boolean isOn = false;
    public double velocityRPM = 0.0;
    public double tempCelcius = 0.0;
    public double currentAmps = 0.0;
    public double positionDeg = 0.0;
  }

  // public void setSetpoint(double setpointRPM);

  public void setVoltage(double volts);

  // public boolean atTarget();

  public void updateInputs(FeederInputsAutoLogged inputs);
}
