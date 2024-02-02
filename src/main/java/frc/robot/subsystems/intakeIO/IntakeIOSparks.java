package frc.robot.subsystems.intakeIO;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
public class IntakeIOSparks implements IntakeIO{

    private TimeOfFlight sensor;
    private CANSparkMax motor;
    public IntakeIOSparks()
    {
        this.sensor = new TimeOfFlight(Constants.RobotMap.INTAKE_TOF_SENSOR_ID);
        this.sensor.setRangingMode(RangingMode.Medium, 24);
        this.motor = new CANSparkMax(Constants.RobotMap.INTAKE_MOTOR_CAN_ID,MotorType.kBrushless);
    }

    @Override
    public void setCurrentLimit(int currentLimit) {
        motor.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.outputVoltage = MathUtil.clamp(motor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.isOn =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity()) > 0.005;

    inputs.velocityRPM = motor.getEncoder().getVelocity();

    inputs.tempCelcius = motor.getMotorTemperature();

    inputs.currentAmps = motor.getOutputCurrent();

    inputs.positionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getPosition());

    inputs.sensorRange = sensor.getRange();
    inputs.sensorStatus = sensor.getStatus().toString();

    //inputs.encoderVelocityRight = cubeSensor.getVelocity();
    //inputs.encoderVoltageRight = cubeSensor.getVoltage();

    //inputs.encoderPositionLeft = coneSensor.getPosition();
    //inputs.encoderVelocityLeft = coneSensor.getVelocity();
    //inputs.encoderVoltageLeft = coneSensor.getVoltage();
    }

    @Override
    public void setTopVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTopVoltage'");
    }

    @Override
    public void setBottomVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setBottomVoltage'");
    }

    @Override
    public boolean hasGamepiece() {
        return (sensor.getRange() < Constants.IntakeConstants.SENSOR_THRESHOLD);
    }
    
}
