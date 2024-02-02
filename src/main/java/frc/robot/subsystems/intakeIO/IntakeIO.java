package frc.robot.subsystems.intakeIO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO
{
    @AutoLog
    public static class IntakeInputs
    {
        public double outputVoltage = 0.0;
        public boolean isOn = false;
        public double velocityRPM = 0.0;
        public double tempCelcius = 0.0;
        public double currentAmps = 0.0;

        public double positionRad = 0.0;
        public double sensorRange = 0.0;
        public String sensorStatus = "";
    }  

    public void setCurrentLimit(int currentLimit);

    public void updateInputs(IntakeInputs inputs);
    
    public void setTopVoltage(double volts);

    public void setBottomVoltage(double volts);

    public boolean hasGamepiece();

}
