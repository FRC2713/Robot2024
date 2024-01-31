package frc.robot.subsystems.shooterIO;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs
    {
        public double outputVoltage = 0.0;
        public double leftFlyWheelSpeedRPM = 0.0;
        public double rightFlyWheelSpeedRPM = 0.0;
        public double leftFLyWheelDrawAmp = 0;
        public double rightFLyWheelDrawAmp = 0;
        public double leftTempCelcius = 0.0;
        public double rightTempCelcius = 0.0;
    }
    public void updateInputs(ShooterInputs inputs);
    public void setLeftVoltage(double voltage);
    public void setRightVoltage(double voltage);
    public void setLeftMotorRPMSetPoint(double rPM);
    public void setRightMotorRPMSetPoint(double rPM);
}
