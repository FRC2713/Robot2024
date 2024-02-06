package frc.robot.subsystems.intakeIO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double leftOutputVoltage = 0.0;
    public boolean leftIsOn = false;
    public double leftVelocityRPM = 0.0;
    public double leftTempCelcius = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftPositionRad = 0.0;

    public double rightOutputVoltage = 0.0;
    public boolean rightIsOn = false;
    public double rightVelocityRPM = 0.0;
    public double rightTempCelcius = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightPositionRad = 0.0;

    public double sensorRange = 0.0;
    public String sensorStatus = "";
  }

  public void updateInputs(IntakeInputsAutoLogged inputs);

  public void setCurrentLimit(int currentLimit);

  public void setVoltage(double leftVolts, double rightVolts);

  public boolean hasGamepiece();
}
